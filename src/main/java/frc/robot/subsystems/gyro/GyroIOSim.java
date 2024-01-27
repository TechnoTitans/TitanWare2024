package frc.robot.subsystems.gyro;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.sim.Pigeon2SimState;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import frc.robot.subsystems.drive.Swerve;
import frc.robot.subsystems.drive.SwerveModule;
import org.littletonrobotics.junction.Logger;

import java.util.Queue;
import java.util.concurrent.locks.ReentrantReadWriteLock;

public class GyroIOSim implements GyroIO {
    public static final double USE_SIMULATED_PITCH = 0;
    public static final double USE_SIMULATED_ROLL = 0;

    private final Pigeon2 pigeon;
    private final Pigeon2SimState pigeonSimState;
    private final SwerveDriveKinematics kinematics;
    private final SwerveModule[] swerveModules;
    private final double[] lastSwerveModulePositionMeters = {0.0, 0.0, 0.0, 0.0};

    private Pose2d gyroUseOdometryPose = new Pose2d(0, 0, Rotation2d.fromDegrees(0));

    // Cached StatusSignals
    private final StatusSignal<Double> _yaw;
    private final StatusSignal<Double> _pitch;
    private final StatusSignal<Double> _roll;
    private final StatusSignal<Double> _yawVelocity;
    private final StatusSignal<Double> _pitchVelocity;
    private final StatusSignal<Double> _rollVelocity;
    private final StatusSignal<Boolean> _faultHardware;

    // StatusSignal queues for high-freq odometry and read lock
    private final ReentrantReadWriteLock signalQueueReadWriteLock;
    private final Queue<Double> timestampQueue;
    private final Queue<Double> yawSignalQueue;

    public GyroIOSim(
            final Pigeon2 pigeon,
            final SwerveDriveKinematics kinematics,
            final Swerve.OdometryThreadRunner odometryThreadRunner,
            final SwerveModule[] swerveModules
    ) {
        this.pigeon = pigeon;
        this.pigeonSimState = pigeon.getSimState();
        this.kinematics = kinematics;
        this.swerveModules = swerveModules;

        this._yaw = pigeon.getYaw();
        this._pitch = pigeon.getPitch();
        this._roll = pigeon.getRoll();
        this._yawVelocity = pigeon.getAngularVelocityZWorld();
        this._pitchVelocity = pigeon.getAngularVelocityXWorld();
        this._rollVelocity = pigeon.getAngularVelocityYWorld();
        this._faultHardware = pigeon.getFault_Hardware();

        this.signalQueueReadWriteLock = odometryThreadRunner.getSignalQueueReadWriteLock();
        this.timestampQueue = odometryThreadRunner.makeTimestampQueue();
        this.yawSignalQueue = odometryThreadRunner.registerSignal(pigeon, _yaw);

        pigeonSimState.setSupplyVoltage(12);
        pigeonSimState.setPitch(USE_SIMULATED_PITCH);
        pigeonSimState.setRoll(USE_SIMULATED_ROLL);
    }

    private void updateGyro() {
        final SwerveModulePosition[] wheelDeltas = new SwerveModulePosition[swerveModules.length];
        for (int i = 0; i < swerveModules.length; i++) {
            final SwerveModulePosition currentSwervePosition = swerveModules[i].getPosition();

            wheelDeltas[i] = new SwerveModulePosition(
                    (currentSwervePosition.distanceMeters - lastSwerveModulePositionMeters[i]),
                    currentSwervePosition.angle
            );
            lastSwerveModulePositionMeters[i] = currentSwervePosition.distanceMeters;
        }

        final Twist2d wheelDeltasTwist = kinematics.toTwist2d(wheelDeltas);
        gyroUseOdometryPose = gyroUseOdometryPose.exp(wheelDeltasTwist);

        Logger.recordOutput(Gyro.logKey + "/GyroUseOdometryPose", gyroUseOdometryPose);
        Logger.recordOutput(Gyro.logKey + "/WheelDeltasTwistDx", wheelDeltasTwist.dx);
        Logger.recordOutput(Gyro.logKey + "/WheelDeltasTwistDy", wheelDeltasTwist.dy);
        Logger.recordOutput(Gyro.logKey + "/WheelDeltasTwistDTheta", wheelDeltasTwist.dtheta);
        Logger.recordOutput(Gyro.logKey + "/LastSwerveModulePositionMeters", lastSwerveModulePositionMeters);

        setAngleInternal(gyroUseOdometryPose.getRotation().getDegrees());
    }

    @SuppressWarnings("DuplicatedCode")
    @Override
    public void config() {
        //TODO fill in correct mount pose
        final Pigeon2Configuration pigeon2Configuration = new Pigeon2Configuration();
        pigeon2Configuration.MountPose.MountPoseYaw = 0;
        pigeon.getConfigurator().apply(pigeon2Configuration);

        BaseStatusSignal.setUpdateFrequencyForAll(
                100,
                _pitch,
                _pitchVelocity,
                _roll,
                _rollVelocity
        );
        BaseStatusSignal.setUpdateFrequencyForAll(
                4,
                _faultHardware
        );
        ParentDevice.optimizeBusUtilizationForAll(pigeon);
    }

    @Override
    public void periodic() {
        updateGyro();
    }

    @SuppressWarnings("DuplicatedCode")
    @Override
    public void updateInputs(final GyroIOInputs inputs) {
        BaseStatusSignal.refreshAll(
                _yaw,
                _pitch,
                _roll,
                _yawVelocity,
                _pitchVelocity,
                _rollVelocity,
                _faultHardware
        );

        inputs.yawPositionDeg = getYaw();
        inputs.pitchPositionDeg = getPitch();
        inputs.rollPositionDeg = getRoll();
        inputs.yawVelocityDegPerSec = _yawVelocity.getValue();
        inputs.pitchVelocityDegPerSec = _pitchVelocity.getValue();
        inputs.rollVelocityDegPerSec = _rollVelocity.getValue();
        inputs.hasHardwareFault = _faultHardware.getValue();

        inputs.odometryTimestampsSec = timestampQueue.stream().mapToDouble(time -> time).toArray();
        timestampQueue.clear();

        inputs.odometryYawPositionsDeg = yawSignalQueue.stream().mapToDouble(yaw -> yaw).toArray();
        yawSignalQueue.clear();
    }

    public double getYaw() {
        // TODO: fairly sure that yaw, pitch, roll velocity don't work in sim, so we can't latency compensate
//        return Phoenix6Utils.latencyCompensateIfSignalIsGood(
//                _yaw,
//                getYawVelocitySignal()
//        );
        return _yaw.getValue();
    }

    public double getPitch() {
        // see previous mention of velocities not working in sim

//        return Phoenix6Utils.latencyCompensateIfSignalIsGood(
//                pigeon.getPitch(),
//                getPitchVelocitySignal()
//        );
        return _pitch.getValue();
    }

    public double getRoll() {
        // see previous mention of velocities not working in sim

//        return Phoenix6Utils.latencyCompensateIfSignalIsGood(
//                pigeon.getRoll(),
//                getRollVelocitySignal()
//        );
        return _roll.getValue();
    }

    private void setAngleInternal(final double angle) {
        pigeonSimState.setRawYaw(angle);
    }

    @Override
    public void setAngle(final Rotation2d angle) {
        pigeon.setYaw(angle.getDegrees());
    }
}
