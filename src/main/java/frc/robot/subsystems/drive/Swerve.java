package frc.robot.subsystems.drive;

import com.ctre.phoenix6.*;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructSerializable;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.constants.HardwareConstants;
import frc.robot.constants.RobotMap;
import frc.robot.subsystems.gyro.*;
import frc.robot.utils.gyro.GyroUtils;
import frc.robot.utils.logging.LogUtils;
import frc.robot.utils.teleop.ControllerUtils;
import frc.robot.utils.teleop.Profiler;
import org.littletonrobotics.junction.Logger;

import java.nio.ByteBuffer;
import java.util.*;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.locks.ReentrantReadWriteLock;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;

public class Swerve extends SubsystemBase {
    protected static final String logKey = "Swerve";
    protected static final String odometryLogKey = "Odometry";

    private Gyro gyro;
    private final GyroIOInputsAutoLogged gyroInputs;
    private final SwerveDriveKinematics kinematics;
    private final SwerveDrivePoseEstimator poseEstimator;

    private final SwerveModule frontLeft, frontRight, backLeft, backRight;
    private final SwerveModule[] swerveModules;

    private final OdometryThreadRunner odometryThreadRunner;

    private final ReentrantReadWriteLock signalQueueReadWriteLock = new ReentrantReadWriteLock();

    public static class OdometryThreadRunner {
        // Increase the priority to dedicate more resources towards running the thread at the right frequency, 1 is the
        // minimum realtime priority and should work well enough
        protected static final int STARTING_THREAD_PRIORITY = 1;
        // 250Hz should work well when on a CAN-FD network, if only on CAN 2.0 or other, this should probably be
        // set to <= 100Hz
        protected static final double UPDATE_FREQUENCY_HZ = 250;

        protected final ReentrantReadWriteLock signalQueueReadWriteLock;
        protected final ReentrantReadWriteLock signalReadWriteLock = new ReentrantReadWriteLock();
        protected final ReentrantReadWriteLock controlReqReadWriteLock = new ReentrantReadWriteLock();

        protected final List<StatusSignal<Double>> allSignals = new ArrayList<>();
        protected final List<Boolean> isLatencyCompensated = new ArrayList<>();
        protected final Map<ParentDevice, ControlRequest> outerAppliedControlReqs = new HashMap<>();
        protected final Map<ParentDevice, ControlRequest> innerAppliedControlReqs = new HashMap<>();
        protected final Map<ParentDevice, Consumer<ControlRequest>> controlReqAppliers = new HashMap<>();
        protected final List<Queue<Double>> queues = new ArrayList<>();

        protected final Thread thread;
        protected final State state = new State();
        protected volatile boolean running = false;

        protected final MedianFilter peakRemover = new MedianFilter(3);
        protected final LinearFilter lowPass = LinearFilter.movingAverage(50);
        protected double lastTimeSeconds = 0;
        protected double currentTimeSeconds = 0;
        protected double averageLoopTimeSeconds = 0;

        protected int failedDAQs = 0;

        protected int lastThreadPriority = STARTING_THREAD_PRIORITY;
        protected volatile int threadPriorityToSet = lastThreadPriority;

        public OdometryThreadRunner(final ReentrantReadWriteLock signalQueueReadWriteLock) {
            this.thread = new Thread(this::run);
            this.thread.setDaemon(true);

            this.signalQueueReadWriteLock = signalQueueReadWriteLock;
        }

        /**
         * Starts the odometry thread.
         */
        public void start() {
            running = true;
            thread.start();
        }

        /**
         * Stops the odometry thread.
         */
        public void stop() {
            stop(0);
        }

        /**
         * Stops the odometry thread with a timeout.
         * @param timeoutMillis The time to wait in milliseconds
         */
        public void stop(final long timeoutMillis) {
            running = false;
            try {
                thread.join(timeoutMillis);
            } catch (final InterruptedException interruptedException) {
                Thread.currentThread().interrupt();
            }
        }

        /**
         * Gets a reference to the signal queue {@link ReentrantReadWriteLock} used to lock
         * read/write operations on any signal queue
         * @return the {@link ReentrantReadWriteLock}
         */
        public ReentrantReadWriteLock getSignalQueueReadWriteLock() {
            return signalQueueReadWriteLock;
        }

        /**
         * Sets the DAQ thread priority to a real time priority under the specified priority level
         * @param priority Priority level to set the DAQ thread to. This is a value between 0 and 99,
         *                 with 99 indicating higher priority and 0 indicating lower priority.
         */
        public void setThreadPriority(int priority) {
            threadPriorityToSet = priority;
        }

        public static class State implements StructSerializable {
            public static final StateStruct struct = new StateStruct();
            public boolean running;
            public int failedDAQs;
            public int statusCode;
            public int maxQueueSize;
            public double odometryPeriodSeconds;

            public static class StateStruct implements Struct<State> {
                @Override
                public Class<State> getTypeClass() {
                    return State.class;
                }

                @Override
                public String getTypeString() {
                    return "struct:Swerve.OdometryThreadRunner.State";
                }

                @Override
                public int getSize() {
                    return kSizeBool + (kSizeInt32 * 4) + kSizeDouble;
                }

                @Override
                public String getSchema() {
                    return "bool running;int32 failedDAQs;int32 statusCode;int32 maxQueueSize;double odometryPeriodSeconds";
                }

                @Override
                public State unpack(final ByteBuffer bb) {
                    final boolean running = bb.get() != 0;
                    final int failedDAQs = bb.getInt();
                    final int statusCode = bb.getInt();
                    final int maxQueueSize = bb.getInt();
                    final double odometryPeriod = bb.getDouble();

                    final State state = new State();
                    state.running = running;
                    state.failedDAQs = failedDAQs;
                    state.statusCode = statusCode;
                    state.maxQueueSize = maxQueueSize;
                    state.odometryPeriodSeconds = odometryPeriod;
                    return state;
                }

                @Override
                public void pack(final ByteBuffer bb, final State value) {
                    bb.put((byte)(value.running ? 1 : 0));
                    bb.putInt(value.failedDAQs);
                    bb.putInt(value.statusCode);
                    bb.putInt(value.maxQueueSize);
                    bb.putDouble(value.odometryPeriodSeconds);
                }
            }
        }

        /**
         * Gets the current state of the {@link OdometryThreadRunner}, describing failed DAQs,
         * actual (measured) odometry period, etc...
         * @return the internal {@link State}
         */
        public State getState() {
            try {
                signalQueueReadWriteLock.readLock().lock();
                return state;
            } finally {
                signalQueueReadWriteLock.readLock().unlock();
            }
        }

        public record Signal<T>(
                boolean isLatencyCompensated,
                StatusSignal<T> baseSignal,
                StatusSignal<T> latencyCompensatorSignal
        ) {
            public static <T> Signal<T> single(final StatusSignal<T> baseSignal) {
                return new Signal<>(false, baseSignal, null);
            }

            public static <T> Signal<T> latencyCompensated(
                    final StatusSignal<T> baseSignal,
                    final StatusSignal<T> latencyCompensatorSignal
            ) {
                return new Signal<>(true, baseSignal, latencyCompensatorSignal);
            }
        }

        public Queue<Double> registerSignal(
                final ParentDevice device,
                final StatusSignal<Double> baseSignal,
                final StatusSignal<Double> latencyCompensatorSignal
        ) {
            return registerSignal(device, Signal.latencyCompensated(baseSignal, latencyCompensatorSignal));
        }

        public Queue<Double> registerSignal(
                final ParentDevice device,
                final Signal<Double> signal
        ) {
            final Queue<Double> queue = new ArrayBlockingQueue<>(100);
            try {
                signalReadWriteLock.writeLock().lock();
                if (!CANBus.isNetworkFD(device.getNetwork())) {
                    throw new RuntimeException("Attempted to register signal from a non CAN-FD device! This is a bug!");
                }

                allSignals.add(signal.baseSignal);
                isLatencyCompensated.add(signal.isLatencyCompensated);
                if (signal.isLatencyCompensated) {
                    allSignals.add(signal.latencyCompensatorSignal);
                    isLatencyCompensated.add(true);
                }

                queues.add(queue);
            } finally {
                signalReadWriteLock.writeLock().unlock();
            }

            return queue;
        }

        public void registerControlRequest(
                final ParentDevice device,
                final ControlRequest controlRequest,
                final Consumer<ControlRequest> applyControlReq
        ) {
            if (outerAppliedControlReqs.containsKey(device)) {
                throw new RuntimeException(String.format(
                        "Attempted to register a ControlRequest for the same device" +
                                "ID: %d (%s) more than once!", device.getDeviceID(), device.getNetwork()
                ));
            }

            outerAppliedControlReqs.put(device, controlRequest);
            try {
                controlReqReadWriteLock.writeLock().lock();

                innerAppliedControlReqs.put(device, controlRequest);
                controlReqAppliers.put(device, applyControlReq);
            } finally {
                controlReqReadWriteLock.writeLock().unlock();
            }
        }

        public void updateControlRequest(final ParentDevice device, final ControlRequest controlRequest) {
            if (outerAppliedControlReqs.get(device) == controlRequest) {
                return;
            }

            outerAppliedControlReqs.put(device, controlRequest);
            try {
                controlReqReadWriteLock.writeLock().lock();
                innerAppliedControlReqs.put(device, controlRequest);
            } finally {
                controlReqReadWriteLock.writeLock().unlock();
            }
        }

        private void run() {
            if (allSignals.isEmpty()) {
                DriverStation.reportError(
                        "Attempted to start OdometryThread without any registered signals!",
                        false
                );
                stop();

                return;
            }

            final BaseStatusSignal[] allSignalsArray = allSignals.toArray(BaseStatusSignal[]::new);
            BaseStatusSignal.setUpdateFrequencyForAll(UPDATE_FREQUENCY_HZ, allSignalsArray);
            Threads.setCurrentThreadPriority(true, STARTING_THREAD_PRIORITY);

            while (running) {
                final int statusCodeValue;
                try {
                    signalReadWriteLock.writeLock().lock();
                    final StatusCode statusCode = BaseStatusSignal.waitForAll(
                            2.0 / UPDATE_FREQUENCY_HZ, allSignalsArray
                    );

                    statusCodeValue = statusCode.value;
                    if (!statusCode.isOK()) {
                        failedDAQs++;
                    }
                } finally {
                    signalReadWriteLock.writeLock().unlock();
                }

                try {
                    signalQueueReadWriteLock.writeLock().lock();
                    lastTimeSeconds = currentTimeSeconds;
                    currentTimeSeconds = Utils.getCurrentTimeSeconds();

                    // We don't care about the peaks, as they correspond to GC events,
                    // and we want the period generally low passed
                    averageLoopTimeSeconds = lowPass.calculate(
                            peakRemover.calculate(currentTimeSeconds - lastTimeSeconds)
                    );

                    state.running = running;
                    state.failedDAQs = failedDAQs;
                    state.statusCode = statusCodeValue;
                    state.odometryPeriodSeconds = averageLoopTimeSeconds;

                    int queueIndex = 0;
                    int maxQueueSize = 0;
                    for (int i = 0; i < allSignals.size(); i++) {
                        final Queue<Double> queue = queues.get(queueIndex);
                        final boolean isLatencyCompensatedSignal = isLatencyCompensated.get(i);

                        if (isLatencyCompensatedSignal) {
                            queue.offer(BaseStatusSignal.getLatencyCompensatedValue(
                                    allSignals.get(i),
                                    allSignals.get(i + 1)
                            ));
                            // skip next signal, as the next signal is the latency compensator for the current signal
                            i++;
                        } else {
                            queue.offer(allSignals.get(i).getValue());
                        }

                        final int queueSize = queue.size();
                        if (queueSize > maxQueueSize) {
                            maxQueueSize = queueSize;
                        }

                        queueIndex++;
                    }

                    state.maxQueueSize = maxQueueSize;
                } finally {
                    signalQueueReadWriteLock.writeLock().unlock();
                }

                try {
                    controlReqReadWriteLock.readLock().lock();
                    for (final Map.Entry<ParentDevice, Consumer<ControlRequest>>
                            controlReqApplierEntry : controlReqAppliers.entrySet()
                    ) {
                        controlReqApplierEntry
                                .getValue()
                                .accept(innerAppliedControlReqs.get(controlReqApplierEntry.getKey()));
                    }
                } finally {
                    controlReqReadWriteLock.readLock().unlock();
                }

                // This is inherently synchronous, since lastThreadPriority is only written
                // here and threadPriorityToSet is only read here
                if (threadPriorityToSet != lastThreadPriority) {
                    Threads.setCurrentThreadPriority(true, threadPriorityToSet);
                    lastThreadPriority = threadPriorityToSet;
                }
            }

            state.running = running;
        }
    }

    Swerve(
            final Gyro gyro,
            final SwerveModule frontLeft,
            final SwerveModule frontRight,
            final SwerveModule backLeft,
            final SwerveModule backRight,
            final SwerveDriveKinematics kinematics,
            final SwerveDrivePoseEstimator poseEstimator,
            final OdometryThreadRunner odometryThreadRunner
    ) {
        this.odometryThreadRunner = odometryThreadRunner;

        this.frontLeft = frontLeft;
        this.frontRight = frontRight;
        this.backLeft = backLeft;
        this.backRight = backRight;

        this.swerveModules = new SwerveModule[]{frontLeft, frontRight, backLeft, backRight};
        this.kinematics = kinematics;

        this.gyroInputs = new GyroIOInputsAutoLogged();
        this.gyro = gyro;

        this.poseEstimator = poseEstimator;
        this.odometryThreadRunner.start();
    }

    public Swerve(
            final Constants.RobotMode mode,
            final HardwareConstants.SwerveModuleConstants frontLeftConstants,
            final HardwareConstants.SwerveModuleConstants frontRightConstants,
            final HardwareConstants.SwerveModuleConstants backLeftConstants,
            final HardwareConstants.SwerveModuleConstants backRightConstants
    ) {
        this.odometryThreadRunner = new OdometryThreadRunner(signalQueueReadWriteLock);

        this.frontLeft = frontLeftConstants.create(mode, odometryThreadRunner);
        this.frontRight = frontRightConstants.create(mode, odometryThreadRunner);
        this.backLeft = backLeftConstants.create(mode, odometryThreadRunner);
        this.backRight = backRightConstants.create(mode, odometryThreadRunner);

        this.swerveModules = new SwerveModule[]{frontLeft, frontRight, backLeft, backRight};
        this.kinematics = new SwerveDriveKinematics(
                frontLeftConstants.translationOffset(),
                frontRightConstants.translationOffset(),
                backLeftConstants.translationOffset(),
                backRightConstants.translationOffset()
        );

        final Pigeon2 pigeon2 = new Pigeon2(RobotMap.Pigeon, RobotMap.CanivoreCANBus);
        this.gyroInputs = new GyroIOInputsAutoLogged();
        this.gyro = switch (mode) {
            case REAL -> new Gyro(new GyroIOPigeon2(pigeon2, odometryThreadRunner), pigeon2);
            case SIM -> new Gyro(new GyroIOSim(pigeon2, kinematics, odometryThreadRunner, swerveModules), pigeon2);
            case REPLAY -> new Gyro(new GyroIO() {
            }, pigeon2);
        };

        //TODO add vision
        this.poseEstimator = new SwerveDrivePoseEstimator(
                kinematics,
                getYaw(),
                getModulePositions(),
                new Pose2d()
//                Constants.Vision.STATE_STD_DEVS,
//                Constants.Vision.VISION_MEASUREMENT_STD_DEVS
        );

        this.odometryThreadRunner.start();
    }

    /**
     * <p>
     * Takes in raw SwerveModuleStates and converts them such that the velocity components are converted to
     * their magnitudes (all positive) and the rotational components are all clamped to [0, 180]
     * </p>
     * <p>
     * This ensures that graphics/displays remain correct/easier to understand when under the case where
     * SwerveModuleStates are optimized and may be 180 degrees off (and the velocity component is negated)
     * </p>
     * <p>
     * Note: Do <b>NOT</b> use this for anything other than displaying SwerveModuleStates
     * </p>
     *
     * @param swerveModuleStates raw SwerveModuleStates retrieved directly from the modules
     * @return modified SwerveModuleStates
     */
    private SwerveModuleState[] modifyModuleStatesForDisplay(final SwerveModuleState[] swerveModuleStates) {
        for (int i = 0; i < swerveModuleStates.length; i++) {
            final SwerveModuleState origLastState = swerveModuleStates[i];
            final double origRots = origLastState.angle.getRotations();

            swerveModuleStates[i] = new SwerveModuleState(
                    Math.abs(origLastState.speedMetersPerSecond),
                    Rotation2d.fromRotations(MathUtil.inputModulus(origRots, 0, 1))
            );
        }

        return swerveModuleStates;
    }

    @Override
    public void periodic() {
        final double swervePeriodicUpdateStart = Logger.getRealTimestamp();
        gyro.periodic();

        frontLeft.periodic();
        frontRight.periodic();
        backLeft.periodic();
        backRight.periodic();

        Logger.recordOutput(
                logKey + "/PeriodicIOPeriodMs",
                LogUtils.microsecondsToMilliseconds(Logger.getRealTimestamp() - swervePeriodicUpdateStart)
        );

        Logger.recordOutput(
                logKey + "/OdometryThreadState",
                OdometryThreadRunner.State.struct,
                odometryThreadRunner.getState()
        );

        //log current swerve chassis speeds
        final ChassisSpeeds robotRelativeSpeeds = getRobotRelativeSpeeds();
        Logger.recordOutput(
                logKey + "/LinearSpeedMetersPerSecond",
                Math.hypot(robotRelativeSpeeds.vxMetersPerSecond, robotRelativeSpeeds.vyMetersPerSecond)
        );
        Logger.recordOutput(logKey + "/RobotRelativeChassisSpeeds", robotRelativeSpeeds);
        Logger.recordOutput(logKey + "/FieldRelativeChassisSpeeds", getFieldRelativeSpeeds());

        //prep states for display
        final SwerveModuleState[] lastDesiredStates = modifyModuleStatesForDisplay(getModuleLastDesiredStates());
        final SwerveModuleState[] currentStates = modifyModuleStatesForDisplay(getModuleStates());

        Logger.recordOutput(logKey + "/DesiredStates", lastDesiredStates);
        Logger.recordOutput(logKey + "/CurrentStates", currentStates);

        // only update gyro from wheel odometry if we're not simulating and the gyro has failed
        if (Constants.CURRENT_MODE == Constants.RobotMode.REAL && gyroInputs.hasHardwareFault && gyro.isReal()) {
            final Pigeon2 pigeon2 = gyro.getPigeon();
            gyro = new Gyro(new GyroIOSim(pigeon2, kinematics, odometryThreadRunner, swerveModules), pigeon2);
        }

        Logger.recordOutput(
               logKey + "/IsUsingFallbackSimGyro",
               Constants.CURRENT_MODE == Constants.RobotMode.REAL && !gyro.isReal()
        );

        // Update PoseEstimator and Odometry
        final double odometryUpdateStart = Logger.getRealTimestamp();
        final Pose2d estimatedPosition = poseEstimator.update(getYaw(), getModulePositions());
        final double odometryUpdatePeriodMs = LogUtils.microsecondsToMilliseconds(
                Logger.getRealTimestamp() - odometryUpdateStart
        );

        Logger.recordOutput(
                odometryLogKey + "/OdometryUpdatePeriodMs", odometryUpdatePeriodMs
        );
        Logger.recordOutput(odometryLogKey + "/Robot2d", estimatedPosition);
        Logger.recordOutput(odometryLogKey + "/Robot3d", GyroUtils.robotPose2dToPose3dWithGyro(
                estimatedPosition,
                GyroUtils.rpyToRotation3d(getRoll(), getPitch(), getYaw())
        ));
    }

    public SwerveDriveKinematics getKinematics() {
        return kinematics;
    }
    public SwerveDrivePoseEstimator getPoseEstimator() {
        return poseEstimator;
    }

    /**
     * Get the estimated {@link Pose2d} of the robot from the {@link SwerveDrivePoseEstimator}.
     * @return the estimated position of the robot, as a {@link Pose2d}
     */
    public Pose2d getEstimatedPosition() {
        return poseEstimator.getEstimatedPosition();
    }

    public OdometryThreadRunner getOdometryThreadRunner() {
        return odometryThreadRunner;
    }

    public Gyro getGyro() {
        return gyro;
    }

    public Rotation2d getPitch() {
        return gyro.getPitchRotation2d();
    }

    public Rotation2d getRoll() {
        return gyro.getRollRotation2d();
    }

    public Rotation2d getYaw() {
        return gyro.getYawRotation2d();
    }

    /**
     * @see Gyro#setAngle(Rotation2d)
     */
    public void setAngle(final Rotation2d angle) {
        gyro.setAngle(angle);
    }

    //TODO add vision
    public void zeroRotation() {
//    public void zeroRotation(final PhotonVision photonVision) {
        gyro.zeroRotation();
//        photonVision.resetPosition(
//                photonVision.getEstimatedPosition(),
//                Rotation2d.fromDegrees(0)
//        );
    }

    public Command zeroRotationCommand() {
        return runOnce(this::zeroRotation);
    }

    public ChassisSpeeds getRobotRelativeSpeeds() {
        return kinematics.toChassisSpeeds(
                    frontLeft.getState(),
                    frontRight.getState(),
                    backLeft.getState(),
                    backRight.getState()
        );
    }

    public ChassisSpeeds getFieldRelativeSpeeds() {
        return ChassisSpeeds.fromFieldRelativeSpeeds(
                getRobotRelativeSpeeds(),
                getYaw().times(-1)
        );
    }

    public SwerveModuleState[] getModuleStates() {
        return new SwerveModuleState[] {
                frontLeft.getState(),
                frontRight.getState(),
                backLeft.getState(),
                backRight.getState()
        };
    }

    public SwerveModuleState[] getModuleLastDesiredStates() {
        return new SwerveModuleState[] {
                frontLeft.getLastDesiredState(),
                frontRight.getLastDesiredState(),
                backLeft.getLastDesiredState(),
                backRight.getLastDesiredState()
        };
    }

    public SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[] {
                frontLeft.getPosition(),
                frontRight.getPosition(),
                backLeft.getPosition(),
                backRight.getPosition()
        };
    }

    public void drive(final SwerveModuleState[] states) {
        SwerveDriveKinematics.desaturateWheelSpeeds(states, Constants.Swerve.Modules.MODULE_MAX_SPEED_M_PER_SEC);
        frontLeft.setDesiredState(states[0]);
        frontRight.setDesiredState(states[1]);
        backLeft.setDesiredState(states[2]);
        backRight.setDesiredState(states[3]);
    }

    public void drive(
            final double xSpeed,
            final double ySpeed,
            final double rot,
            final boolean fieldRelative
    ) {
        final ChassisSpeeds speeds = fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, getYaw())
                : new ChassisSpeeds(xSpeed, ySpeed, rot);

        drive(speeds);
    }

    public void drive(final ChassisSpeeds speeds) {
        // TODO: maybe replace with ChassisSpeeds.discretize() or some other tyler math that's more
        //  "mathematically correct"
        // see https://www.chiefdelphi.com/t/whitepaper-swerve-drive-skew-and-second-order-kinematics/416964/40
        // lookahead 4 loop cycles cause uh the goated teams do it and it works so uh
        final double dtSeconds = 4 * Constants.LOOP_PERIOD_SECONDS;
        final Pose2d desiredDeltaPose = new Pose2d(
                speeds.vxMetersPerSecond * dtSeconds,
                speeds.vyMetersPerSecond * dtSeconds,
                Rotation2d.fromRadians(speeds.omegaRadiansPerSecond * dtSeconds)
        );

        final Twist2d twist2d = new Pose2d().log(desiredDeltaPose);
        final ChassisSpeeds correctedSpeeds = new ChassisSpeeds(
                twist2d.dx / dtSeconds,
                twist2d.dy / dtSeconds,
                twist2d.dtheta / dtSeconds
        );

        drive(kinematics.toSwerveModuleStates(correctedSpeeds));
    }

    public Command teleopDriveCommand(
            final DoubleSupplier xSpeedSupplier,
            final DoubleSupplier ySpeedSupplier,
            final DoubleSupplier rotSupplier
    ) {
        return run(() -> {
            final Profiler.DriverProfile driverProfile = Profiler.getDriverProfile();
            final Profiler.SwerveSpeed swerveSpeed = Profiler.getSwerveSpeed();

            final double throttleWeight = swerveSpeed.getThrottleWeight();
            final double rotWeight = swerveSpeed.getRotateWeight();

            final Translation2d leftStickSpeeds = ControllerUtils.getStickXYSquaredInput(
                    xSpeedSupplier.getAsDouble(),
                    ySpeedSupplier.getAsDouble(),
                    0.01,
                    Constants.Swerve.TELEOP_MAX_SPEED_MPS,
                    driverProfile.getThrottleSensitivity(),
                    throttleWeight
            );

            final double rot = ControllerUtils.getStickSquaredInput(
                    rotSupplier.getAsDouble(),
                    0.01,
                    Constants.Swerve.TELEOP_MAX_ANGULAR_SPEED_RAD_PER_SEC,
                    driverProfile.getRotationalSensitivity(),
                    rotWeight
            );

            drive(
                    leftStickSpeeds.getX(),
                    leftStickSpeeds.getY(),
                    rot,
                    true
            );
        });
    }

    public void stop() {
        drive(new ChassisSpeeds());
    }

    /**
     * Drive all modules to a raw {@link SwerveModuleState}
     * @param s1 speed of module 1 (m/s)
     * @param s2 speed of module 2 (m/s)
     * @param s3 speed of module 3 (m/s)
     * @param s4 speed of module 4 (m/s)
     * @param a1 angle of module 1 (deg)
     * @param a2 angle of module 2 (deg)
     * @param a3 angle of module 3 (deg)
     * @param a4 angle of module 4 (deg)
     * @see Swerve#drive(SwerveModuleState[])
     * @see SwerveModuleState
     */
    public void rawSet(
            final double s1,
            final double s2,
            final double s3,
            final double s4,
            final double a1,
            final double a2,
            final double a3,
            final double a4
    ) {
        drive(new SwerveModuleState[] {
                new SwerveModuleState(s1, Rotation2d.fromDegrees(a1)),
                new SwerveModuleState(s2, Rotation2d.fromDegrees(a2)),
                new SwerveModuleState(s3, Rotation2d.fromDegrees(a3)),
                new SwerveModuleState(s4, Rotation2d.fromDegrees(a4))
        });
    }

    /**
     * Zero all modules
     * @see Swerve#rawSet(double, double, double, double, double, double, double, double)
     */
    @SuppressWarnings("unused")
    public Command zeroCommand() {
        return runOnce(() -> rawSet(0, 0, 0, 0, 0, 0, 0, 0));
    }

    /**
     * Put modules into an X pattern (significantly reduces the swerve's ability to coast/roll)
     * @see Swerve#rawSet(double, double, double, double, double, double, double, double)
     */
    public void wheelX() {
        rawSet(0, 0, 0, 0, 45, -45, -45, 45);
    }

    public Command wheelXCommand() {
        return runOnce(this::wheelX);
    }

    /**
     * Set the desired {@link NeutralModeValue} of all module drive motors
     * @param neutralMode the desired {@link NeutralModeValue}
     * @see SwerveModule#setNeutralMode(NeutralModeValue)
     */
    public void setNeutralMode(final NeutralModeValue neutralMode) {
        frontLeft.setNeutralMode(neutralMode);
        frontRight.setNeutralMode(neutralMode);
        backLeft.setNeutralMode(neutralMode);
        backRight.setNeutralMode(neutralMode);
    }
}
