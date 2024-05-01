package frc.robot.subsystems.drive;

import com.choreo.lib.ChoreoTrajectory;
import com.choreo.lib.ChoreoTrajectoryState;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Current;
import edu.wpi.first.units.Measure;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Robot;
import frc.robot.auto.Autos;
import frc.robot.constants.Constants;
import frc.robot.constants.HardwareConstants;
import frc.robot.subsystems.drive.constants.SwerveConstants;
import frc.robot.subsystems.drive.trajectory.HolonomicChoreoController;
import frc.robot.subsystems.drive.trajectory.HolonomicDriveWithPIDController;
import frc.robot.subsystems.gyro.Gyro;
import frc.robot.utils.gyro.GyroUtils;
import frc.robot.utils.logging.LogUtils;
import frc.robot.utils.teleop.ControllerUtils;
import frc.robot.utils.teleop.Profiler;
import org.littletonrobotics.junction.Logger;

import java.util.concurrent.locks.ReentrantReadWriteLock;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.drive.constants.SwerveConstants.Config;

public class Swerve extends SubsystemBase {
    protected static final String LogKey = "Swerve";
    protected static final String OdometryLogKey = LogKey + "/Odometry";

    private static final HolonomicPathFollowerConfig HolonomicPathFollowerConfig = new HolonomicPathFollowerConfig(
            new PIDConstants(5, 0, 0),
            new PIDConstants(5, 0, 0),
            Config.maxLinearVelocity(),
            Config.driveBaseRadiusMeters(),
            new ReplanningConfig()
    );

    private final Constants.RobotMode mode;

    private Gyro gyro;
    private final HardwareConstants.GyroConstants gyroConstants;
    private final SwerveDriveKinematics kinematics;
    private final SwerveDrivePoseEstimator poseEstimator;

    private final SwerveDriveOdometry wheelOnlyOdometry;
    private Pose2d lastWheelOnlyPose = new Pose2d();

    private final SwerveModule frontLeft, frontRight, backLeft, backRight;
    private final SwerveModule[] swerveModules;

    private final OdometryThreadRunner odometryThreadRunner;
    private final ReentrantReadWriteLock signalQueueReadWriteLock = new ReentrantReadWriteLock();

    private final double maxLinearVelocity = Config.maxLinearVelocity();
    private final double maxAngularVelocity = Config.maxAngularVelocity();

    public final Trigger atHeadingSetpoint;
    private boolean overridePathHeading = false;
    private Supplier<Rotation2d> headingOverrideSupplier = Rotation2d::new;
    private boolean headingControllerActive = false;
    private Rotation2d headingTarget = new Rotation2d();
    private final PIDController headingController;

    public final Trigger atHolonomicDrivePose;
    private boolean holonomicControllerActive = false;
    private Pose2d holonomicPoseTarget = new Pose2d();
    private final HolonomicDriveWithPIDController holonomicDriveWithPIDController;

    private final HolonomicChoreoController choreoController;

    private final SysIdRoutine linearVoltageSysIdRoutine;
    private final SysIdRoutine linearTorqueCurrentSysIdRoutine;
    private final SysIdRoutine angularVoltageSysIdRoutine;

    public Swerve(
            final Constants.RobotMode mode,
            final HardwareConstants.GyroConstants gyroConstants,
            final SwerveConstants.SwerveModuleConstants frontLeftConstants,
            final SwerveConstants.SwerveModuleConstants frontRightConstants,
            final SwerveConstants.SwerveModuleConstants backLeftConstants,
            final SwerveConstants.SwerveModuleConstants backRightConstants
    ) {
        this.mode = mode;
        this.gyroConstants = gyroConstants;
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

        this.gyro = new Gyro(gyroConstants, odometryThreadRunner, kinematics, swerveModules, mode);

        this.poseEstimator = new SwerveDrivePoseEstimator(
                kinematics,
                getGyro().getYawRotation2d(),
                getModulePositions(),
                new Pose2d(),
                Constants.Vision.STATE_STD_DEVS,
                VecBuilder.fill(0.6, 0.6, Units.degreesToRadians(80))
        );

        this.wheelOnlyOdometry = new SwerveDriveOdometry(
                kinematics,
                getGyro().getYawRotation2d(),
                getModulePositions(),
                lastWheelOnlyPose
        );

        this.headingController = new PIDController(4, 0, 0);
        this.headingController.enableContinuousInput(-Math.PI, Math.PI);
        this.headingController.setTolerance(Units.degreesToRadians(3), Units.degreesToRadians(6));
        this.atHeadingSetpoint = new Trigger(
                () -> headingControllerActive &&
                        MathUtil.isNear(
                                headingTarget.getRadians(),
                                getPose().getRotation().getRadians(),
                                Units.degreesToRadians(3)
                        ) &&
                        MathUtil.isNear(
                                0,
                                getRobotRelativeSpeeds().omegaRadiansPerSecond,
                                Units.degreesToRadians(12)
                        )
        );

        this.holonomicDriveWithPIDController = new HolonomicDriveWithPIDController(
                new PIDController(4, 0, 0),
                new PIDController(4, 0, 0),
                new ProfiledPIDController(
                        headingController.getP(), headingController.getI(), headingController.getD(),
                        new TrapezoidProfile.Constraints(
                                Config.maxAngularVelocity() * 0.95,
                                Config.maxAngularAcceleration() * 0.75
                        )
                ),
                new Pose2d(0.05, 0.05, Rotation2d.fromDegrees(6))
        );
        this.atHolonomicDrivePose = new Trigger(holonomicDriveWithPIDController::atReference);

        this.choreoController = new HolonomicChoreoController(
                new PIDController(5, 0, 0),
                new PIDController(5, 0, 0),
                new PIDController(5, 0, 0)
        );

        this.linearVoltageSysIdRoutine = makeLinearVoltageSysIdRoutine();
        this.linearTorqueCurrentSysIdRoutine = makeLinearTorqueCurrentSysIdRoutine();
        this.angularVoltageSysIdRoutine = makeAngularVoltageSysIdRoutine();

        Swerve.configurePathPlannerAutoBuilder(
                this,
                Robot.IsRedAlliance,
                currentPose -> Logger.recordOutput(Autos.LogKey + "/CurrentPose", currentPose),
                targetPose -> Logger.recordOutput(Autos.LogKey + "/TargetPose", targetPose)
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
    private static SwerveModuleState[] modifyModuleStatesForDisplay(final SwerveModuleState[] swerveModuleStates) {
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

    @SuppressWarnings("SameParameterValue")
    private static void configurePathPlannerAutoBuilder(
            final Swerve swerve,
            final BooleanSupplier flipPathSupplier,
            final Consumer<Pose2d> logCurrentPoseConsumer,
            final Consumer<Pose2d> logTargetPoseConsumer
    ) {
        PathPlannerLogging.setLogCurrentPoseCallback(logCurrentPoseConsumer);
        PathPlannerLogging.setLogTargetPoseCallback(logTargetPoseConsumer);
        AutoBuilder.configureHolonomic(
                swerve::getPose,
                swerve::resetPose,
                swerve::getRobotRelativeSpeeds,
                swerve::drive,
                Swerve.HolonomicPathFollowerConfig,
                flipPathSupplier,
                swerve
        );
    }

    @Override
    public void periodic() {
        final double swervePeriodicUpdateStart = Logger.getRealTimestamp();
        try {
            signalQueueReadWriteLock.writeLock().lock();

            gyro.updateInputs();
            frontLeft.updateInputs();
            frontRight.updateInputs();
            backLeft.updateInputs();
            backRight.updateInputs();
        } finally {
            signalQueueReadWriteLock.writeLock().unlock();
        }

        gyro.periodic();
        frontLeft.periodic();
        frontRight.periodic();
        backLeft.periodic();
        backRight.periodic();

        Logger.recordOutput(
                LogKey + "/PeriodicIOPeriodMs",
                LogUtils.microsecondsToMilliseconds(Logger.getRealTimestamp() - swervePeriodicUpdateStart)
        );

        // Update PoseEstimator and Odometry
        final double odometryUpdateStart = Logger.getRealTimestamp();

        // Signals are synchronous, this means that all signals should have observed the same number of timestamps
        final double[] sampleTimestamps = frontLeft.getOdometryTimestamps();
        final double[] gyroYawPositions = gyro.getOdometryYawPositions();
        final int sampleCount = sampleTimestamps.length;
        final int moduleCount = swerveModules.length;

        for (int timestampIndex = 0; timestampIndex < sampleCount; timestampIndex++) {
            final SwerveModulePosition[] positions = new SwerveModulePosition[moduleCount];
            for (int moduleIndex = 0; moduleIndex < moduleCount; moduleIndex++) {
                positions[moduleIndex] = swerveModules[moduleIndex].getOdometryPositions()[timestampIndex];
            }

            wheelOnlyOdometry.update(Rotation2d.fromDegrees(gyroYawPositions[timestampIndex]), positions);
            poseEstimator.updateWithTime(
                    sampleTimestamps[timestampIndex],
                    Rotation2d.fromDegrees(gyroYawPositions[timestampIndex]),
                    positions
            );
        }

        final double odometryUpdatePeriodMs = LogUtils.microsecondsToMilliseconds(
                Logger.getRealTimestamp() - odometryUpdateStart
        );

        Logger.recordOutput(
                LogKey + "/OdometryThreadState",
                OdometryThreadRunner.State.struct,
                odometryThreadRunner.getState()
        );

        //log current swerve chassis speeds
        final ChassisSpeeds robotRelativeSpeeds = getRobotRelativeSpeeds();
        Logger.recordOutput(
                LogKey + "/LinearSpeedMetersPerSecond",
                Math.hypot(robotRelativeSpeeds.vxMetersPerSecond, robotRelativeSpeeds.vyMetersPerSecond)
        );
        Logger.recordOutput(LogKey + "/RobotRelativeChassisSpeeds", robotRelativeSpeeds);
        Logger.recordOutput(LogKey + "/FieldRelativeChassisSpeeds", getFieldRelativeSpeeds());

        //prep states for display
        final SwerveModuleState[] lastDesiredStates = Swerve.modifyModuleStatesForDisplay(getModuleLastDesiredStates());
        final SwerveModuleState[] currentStates = Swerve.modifyModuleStatesForDisplay(getModuleStates());

        Logger.recordOutput(LogKey + "/DesiredStates", lastDesiredStates);
        Logger.recordOutput(LogKey + "/CurrentStates", currentStates);

        // only update gyro from wheel odometry if we're not simulating and the gyro has failed
        if (mode == Constants.RobotMode.REAL && gyro.hasHardwareFault() && gyro.isReal()) {
            gyro = new Gyro(gyroConstants, odometryThreadRunner, kinematics, swerveModules, Constants.RobotMode.SIM);
        }

        Logger.recordOutput(
               LogKey + "/IsUsingFallbackSimGyro",
               mode == Constants.RobotMode.REAL && !gyro.isReal()
        );

        Logger.recordOutput(
                OdometryLogKey + "/OdometryUpdatePeriodMs", odometryUpdatePeriodMs
        );

        final Pose2d wheelOnlyPose = wheelOnlyOdometry.getPoseMeters();
        Logger.recordOutput(OdometryLogKey + "/WheelOnlyRobot2d", wheelOnlyPose);
        Logger.recordOutput(OdometryLogKey + "/WheelOnlyTwist", lastWheelOnlyPose.log(wheelOnlyPose));
        lastWheelOnlyPose = wheelOnlyPose;

        Logger.recordOutput(OdometryLogKey + "/Robot2d", getPose());
        Logger.recordOutput(OdometryLogKey + "/Robot3d", GyroUtils.robotPose2dToPose3dWithGyro(
                getPose(),
                GyroUtils.rpyToRotation3d(getRoll(), getPitch(), getYaw())
        ));

        Logger.recordOutput(LogKey + "/HeadingController/Active", headingControllerActive);
        Logger.recordOutput(LogKey + "/HeadingController/AtHeadingSetpoint", atHeadingSetpoint.getAsBoolean());
        Logger.recordOutput(LogKey + "/HeadingController/TargetHeading", headingTarget);
        Logger.recordOutput(LogKey + "/HeadingController/TargetPose", new Pose2d(
                getPose().getTranslation(),
                headingTarget
        ));

        Logger.recordOutput(LogKey + "/HolonomicController/Active", holonomicControllerActive);
        Logger.recordOutput(LogKey + "/HolonomicController/TargetPose", holonomicPoseTarget);
        Logger.recordOutput(LogKey + "/HolonomicController/AtPoseSetpoint", atHolonomicDrivePose.getAsBoolean());
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
    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
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
        return getPose().getRotation();
    }

    /**
     * @see Gyro#setAngle(Rotation2d)
     */
    public void setAngle(final Rotation2d angle) {
        gyro.setAngle(angle);
    }

    public void zeroRotation() {
        final Rotation2d gyroYaw = gyro.getYawRotation2d();
        final SwerveModulePosition[] positions = getModulePositions();
        final Pose2d pose = new Pose2d(
                getPose().getTranslation(),
                Robot.IsRedAlliance.getAsBoolean()
                        ? Rotation2d.fromDegrees(180)
                        : Rotation2d.fromDegrees(0)
        );

        poseEstimator.resetPosition(gyroYaw, positions, pose);
        wheelOnlyOdometry.resetPosition(gyroYaw, positions, pose);
    }

    public Command zeroRotationCommand() {
        return runOnce(this::zeroRotation);
    }

    public void resetPose(final Pose2d robotPose) {
        final Rotation2d gyroYaw = gyro.getYawRotation2d();
        final SwerveModulePosition[] positions = getModulePositions();

        poseEstimator.resetPosition(gyroYaw, positions, robotPose);
        wheelOnlyOdometry.resetPosition(gyroYaw, positions, robotPose);
    }

    public Command resetPoseCommand(final Pose2d robotPose) {
        return runOnce(() -> resetPose(robotPose));
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
        return ChassisSpeeds.fromRobotRelativeSpeeds(
                getRobotRelativeSpeeds(),
                getYaw()
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
        SwerveDriveKinematics.desaturateWheelSpeeds(states, maxLinearVelocity);
        frontLeft.setDesiredState(states[0]);
        frontRight.setDesiredState(states[1]);
        backLeft.setDesiredState(states[2]);
        backRight.setDesiredState(states[3]);
    }

    public void drive(final SwerveModuleState[] states, final ChassisSpeeds desiredSpeeds) {
        SwerveDriveKinematics.desaturateWheelSpeeds(
                states,
                desiredSpeeds,
                maxLinearVelocity,
                maxLinearVelocity,
                maxAngularVelocity
        );

        frontLeft.setDesiredState(states[0]);
        frontRight.setDesiredState(states[1]);
        backLeft.setDesiredState(states[2]);
        backRight.setDesiredState(states[3]);
    }

    public void drive(
            final double xSpeed,
            final double ySpeed,
            final double omega,
            final boolean fieldRelative,
            final boolean invertYaw
    ) {
        final ChassisSpeeds speeds;
        if (fieldRelative) {
            final Rotation2d poseYaw = getYaw();
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                    xSpeed,
                    ySpeed,
                    omega,
                    invertYaw
                            ? poseYaw.plus(Rotation2d.fromRadians(Math.PI))
                            : poseYaw
            );
        } else {
            speeds = new ChassisSpeeds(xSpeed, ySpeed, omega);
        }

        drive(speeds);
    }

    public void drive(final ChassisSpeeds speeds) {
        final ChassisSpeeds correctedSpeeds = ChassisSpeeds.discretize(
                speeds,
                4 * Constants.LOOP_PERIOD_SECONDS
        );

        drive(kinematics.toSwerveModuleStates(correctedSpeeds), speeds);
    }

    public Command teleopDriveCommand(
            final DoubleSupplier xSpeedSupplier,
            final DoubleSupplier ySpeedSupplier,
            final DoubleSupplier rotSupplier,
            final BooleanSupplier invertYaw
    ) {
        return run(() -> {
            final Profiler.DriverProfile driverProfile = Profiler.getDriverProfile();
            final Profiler.SwerveSpeed swerveSpeed = Profiler.getSwerveSpeed();

            final Translation2d translationInput = ControllerUtils.calculateLinearVelocity(
                    -xSpeedSupplier.getAsDouble(),
                    -ySpeedSupplier.getAsDouble(),
                    0.01
            );

            final double rotationInput = ControllerUtils.getStickSquaredInput(
                    -rotSupplier.getAsDouble(),
                    0.01
            );

            drive(
                    translationInput.getX()
                            * swerveSpeed.getTranslationSpeed()
                            * driverProfile.getTranslationSensitivity(),
                    translationInput.getY()
                            * swerveSpeed.getTranslationSpeed()
                            * driverProfile.getTranslationSensitivity(),
                    rotationInput
                            * swerveSpeed.getRotationSpeed()
                            * driverProfile.getRotationalSensitivity(),
                    true,
                    invertYaw.getAsBoolean()
            );
        });
    }

    public Command teleopFacingAngleCommand(
            final DoubleSupplier xSpeedSupplier,
            final DoubleSupplier ySpeedSupplier,
            final Supplier<Rotation2d> rotationTargetSupplier
    ) {
        return Commands.sequence(
                runOnce(() -> {
                    headingControllerActive = true;
                    headingController.reset();
                }),
                run(() -> {
                    final Profiler.DriverProfile driverProfile = Profiler.getDriverProfile();
                    final Profiler.SwerveSpeed swerveSpeed = Profiler.getSwerveSpeed();

                    final Translation2d translationInput = ControllerUtils.calculateLinearVelocity(
                            -xSpeedSupplier.getAsDouble(),
                            -ySpeedSupplier.getAsDouble(),
                            0.01
                    );

                    this.headingTarget = rotationTargetSupplier.get();
                    drive(
                            translationInput.getX()
                                    * swerveSpeed.getTranslationSpeed()
                                    * driverProfile.getTranslationSensitivity(),
                            translationInput.getY()
                                    * swerveSpeed.getTranslationSpeed()
                                    * driverProfile.getTranslationSensitivity(),
                            headingController.calculate(getYaw().getRadians(), headingTarget.getRadians()),
                            true,
                            Robot.IsRedAlliance.getAsBoolean()
                    );
                })
        ).finallyDo(() -> headingControllerActive = false);
    }

    public Command faceAngle(final Supplier<Rotation2d> rotationTargetSupplier) {
        return Commands.sequence(
                runOnce(() -> {
                    headingControllerActive = true;
                    headingController.reset();
                }),
                run(() -> {
                    this.headingTarget = rotationTargetSupplier.get();
                    drive(
                            0,
                            0,
                            headingController.calculate(getYaw().getRadians(), headingTarget.getRadians()),
                            true,
                            false
                    );
                })
        ).finallyDo(() -> headingControllerActive = false);
    }

    public Command driveToPose(final Supplier<Pose2d> poseSupplier) {
        return Commands.sequence(
                runOnce(() -> {
                    holonomicControllerActive = true;
                    holonomicDriveWithPIDController.reset(getPose(), getRobotRelativeSpeeds());
                }),
                run(() -> {
                    this.holonomicPoseTarget = poseSupplier.get();
                    drive(holonomicDriveWithPIDController.calculate(getPose(), holonomicPoseTarget));
                }).until(holonomicDriveWithPIDController::atReference),
                runOnce(this::stop)
        ).finallyDo(() -> holonomicControllerActive = false);
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
     * @see Swerve#drive(ChassisSpeeds)
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
    public Command zeroModulesCommand() {
        return runOnce(() -> rawSet(0, 0, 0, 0, 0, 0, 0, 0));
    }

    /**
     * Put modules into an X pattern (significantly reduces the swerve's ability to coast/roll)
     * @see Swerve#rawSet(double, double, double, double, double, double, double, double)
     */
    public void wheelX() {
        rawSet(0, 0, 0, 0, 45, -45, -45, 45);
    }

    @SuppressWarnings("unused")
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

    public void setPathHeadingOverride(final Supplier<Rotation2d> headingOverrideSupplier) {
        this.overridePathHeading = true;
        this.headingOverrideSupplier = headingOverrideSupplier;
    }

    public void clearPathHeadingOverride() {
        this.overridePathHeading = false;
    }

    public Command followChoreoPathCommand(
            final ChoreoTrajectory choreoTrajectory,
            final BooleanSupplier mirrorTrajectory
    ) {
        final Timer timer = new Timer();
        return Commands.sequence(
                runOnce(() -> {
                    Logger.recordOutput(
                            Autos.LogKey + "/Trajectory",
                            mirrorTrajectory.getAsBoolean()
                                    ? choreoTrajectory.flipped().getPoses()
                                    : choreoTrajectory.getPoses()
                    );

                    choreoController.reset();
                    timer.restart();
                }),
                run(() -> {
                    final double time = timer.get();
                    final Pose2d currentPose = getPose();
                    final ChoreoTrajectoryState targetState = choreoTrajectory.sample(
                            time,
                            mirrorTrajectory.getAsBoolean()
                    );

                    final Pose2d choreoTargetPose = targetState.getPose();
                    final Rotation2d headingOverride = headingOverrideSupplier.get();
                    final Pose2d targetPose = overridePathHeading
                            ? new Pose2d(choreoTargetPose.getTranslation(), headingOverride)
                            : targetState.getPose();
                    Logger.recordOutput(Autos.LogKey + "/Timestamp", time);
                    Logger.recordOutput(Autos.LogKey + "/CurrentPose", currentPose);
                    Logger.recordOutput(Autos.LogKey + "/TargetSpeeds", targetState.getChassisSpeeds());
                    Logger.recordOutput(Autos.LogKey + "/TargetPose", targetPose);

                    Logger.recordOutput(
                            Autos.LogKey + "/TargetRotation",
                            MathUtil.angleModulus(targetPose.getRotation().getRadians())
                    );

                    Logger.recordOutput(
                            Autos.LogKey + "/CurrentRotation",
                            MathUtil.angleModulus(currentPose.getRotation().getRadians())
                    );

                    if (overridePathHeading) {
                        drive(choreoController.calculate(
                                currentPose,
                                targetState,
                                headingOverride
                        ));
                    } else {
                        drive(choreoController.calculate(currentPose, targetState));
                    }
                })
                        .until(() -> timer.hasElapsed(choreoTrajectory.getTotalTime()))
                        .finallyDo((interrupted) -> {
                            timer.stop();
                            if (interrupted) {
                                drive(new ChassisSpeeds());
                            } else {
                                drive(choreoTrajectory.getFinalState().getChassisSpeeds());
                            }
                        })
        );
    }

//    public Command followPathPlanner() {
//        PathPlannerPath.bezierFromPoses()
//    }

    private SysIdRoutine makeLinearVoltageSysIdRoutine() {
        return new SysIdRoutine(
                new SysIdRoutine.Config(
                        Volts.of(2).per(Second),
                        Volts.of(6),
                        Seconds.of(12),
                        state -> SignalLogger.writeString(LogKey + "-state", state.toString())
                ),
                new SysIdRoutine.Mechanism(
                        voltageMeasure -> {
                            final double volts = voltageMeasure.in(Volts);
                            frontLeft.driveVoltageCharacterization(volts, 0);
                            frontRight.driveVoltageCharacterization(volts, 0);
                            backLeft.driveVoltageCharacterization(volts, 0);
                            backRight.driveVoltageCharacterization(volts, 0);
                        },
                        null,
                        this
                )
        );
    }

    @SuppressWarnings("unused")
    public Command linearVoltageSysIdQuasistaticCommand(final SysIdRoutine.Direction direction) {
        return linearVoltageSysIdRoutine.quasistatic(direction);
    }

    @SuppressWarnings("unused")
    public Command linearVoltageSysIdDynamicCommand(final SysIdRoutine.Direction direction) {
        return linearVoltageSysIdRoutine.dynamic(direction);
    }

    private SysIdRoutine makeLinearTorqueCurrentSysIdRoutine() {
        return new SysIdRoutine(
                new SysIdRoutine.Config(
                        // this is actually amps/sec not volts/sec
                        Volts.of(4).per(Second),
                        // this is actually amps not volts
                        Volts.of(12),
                        Seconds.of(20),
                        state -> SignalLogger.writeString(LogKey + "-state", state.toString())
                ),
                new SysIdRoutine.Mechanism(
                        voltageMeasure -> {
                            // convert the voltage measure to an amperage measure by tricking it
                            final Measure<Current> currentMeasure = Amps.of(voltageMeasure.magnitude());
                            final double amps = currentMeasure.in(Amps);
                            frontLeft.driveTorqueCurrentCharacterization(amps, 0);
                            frontRight.driveTorqueCurrentCharacterization(amps, 0);
                            backLeft.driveTorqueCurrentCharacterization(amps, 0);
                            backRight.driveTorqueCurrentCharacterization(amps, 0);
                        },
                        null,
                        this
                )
        );
    }

    @SuppressWarnings("unused")
    public Command linearTorqueCurrentSysIdQuasistaticCommand(final SysIdRoutine.Direction direction) {
        return linearTorqueCurrentSysIdRoutine.quasistatic(direction);
    }

    @SuppressWarnings("unused")
    public Command linearTorqueCurrentSysIdDynamicCommand(final SysIdRoutine.Direction direction) {
        return linearTorqueCurrentSysIdRoutine.dynamic(direction);
    }

    private SysIdRoutine makeAngularVoltageSysIdRoutine() {
        return new SysIdRoutine(
                new SysIdRoutine.Config(
                        // this is actually amps/sec not volts/sec
                        Volts.of(1).per(Second),
                        Volts.of(10),
                        Seconds.of(20),
                        state -> SignalLogger.writeString("state", state.toString())
                ),
                new SysIdRoutine.Mechanism(
                        voltageMeasure -> {
                            // convert the voltage measure to an amperage measure by tricking it
                            final double volts = voltageMeasure.in(Volts);
                            frontLeft.driveVoltageCharacterization(volts, -0.125);
                            frontRight.driveVoltageCharacterization(volts, 0.625);
                            backLeft.driveVoltageCharacterization(volts, 0.125);
                            backRight.driveVoltageCharacterization(volts, -0.625);
                        },
                        null,
                        this
                )
        );
    }

    @SuppressWarnings("unused")
    public Command angularVoltageSysIdQuasistaticCommand(final SysIdRoutine.Direction direction) {
        return angularVoltageSysIdRoutine.quasistatic(direction);
    }

    @SuppressWarnings("unused")
    public Command angularVoltageSysIdDynamicCommand(final SysIdRoutine.Direction direction) {
        return angularVoltageSysIdRoutine.dynamic(direction);
    }
}
