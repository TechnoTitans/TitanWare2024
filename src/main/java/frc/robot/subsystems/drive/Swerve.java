package frc.robot.subsystems.drive;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.constants.HardwareConstants;
import frc.robot.constants.RobotMap;
import frc.robot.subsystems.gyro.*;
import frc.robot.utils.gyro.GyroUtils;
import frc.robot.utils.logging.LogUtils;
import org.littletonrobotics.junction.Logger;

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

    Swerve(
            final Gyro gyro,
            final SwerveModule frontLeft,
            final SwerveModule frontRight,
            final SwerveModule backLeft,
            final SwerveModule backRight,
            final SwerveDriveKinematics kinematics,
            final SwerveDrivePoseEstimator poseEstimator
    ) {
        this.frontLeft = frontLeft;
        this.frontRight = frontRight;
        this.backLeft = backLeft;
        this.backRight = backRight;

        this.swerveModules = new SwerveModule[] {frontLeft, frontRight, backLeft, backRight};
        this.kinematics = kinematics;

        this.gyroInputs = new GyroIOInputsAutoLogged();
        this.gyro = gyro;

        this.poseEstimator = poseEstimator;
    }

    public Swerve(
            final Constants.RobotMode mode,
            final HardwareConstants.SwerveModuleConstants frontLeftConstants,
            final HardwareConstants.SwerveModuleConstants frontRightConstants,
            final HardwareConstants.SwerveModuleConstants backLeftConstants,
            final HardwareConstants.SwerveModuleConstants backRightConstants
    ) {
        final Pigeon2 pigeon2 = new Pigeon2(RobotMap.Pigeon, RobotMap.CanivoreCANBus);

        this.frontLeft = frontLeftConstants.create(mode);
        this.frontRight = frontRightConstants.create(mode);
        this.backLeft = backLeftConstants.create(mode);
        this.backRight = backRightConstants.create(mode);

        this.swerveModules = new SwerveModule[] {frontLeft, frontRight, backLeft, backRight};
        this.kinematics = new SwerveDriveKinematics(
                frontLeftConstants.translationOffset(),
                frontRightConstants.translationOffset(),
                backLeftConstants.translationOffset(),
                backRightConstants.translationOffset()
        );

        this.gyroInputs = new GyroIOInputsAutoLogged();
        this.gyro = switch (mode) {
            case REAL -> new Gyro(new GyroIOPigeon2(pigeon2), pigeon2);
            case SIM -> new Gyro(new GyroIOSim(pigeon2, kinematics, swerveModules), pigeon2);
            case REPLAY -> new Gyro(new GyroIO() {}, pigeon2);
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
            gyro = new Gyro(new GyroIOSim(pigeon2, kinematics, swerveModules), pigeon2);
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

    public void drive(final SwerveModuleState[] states, final double moduleMaxSpeed) {
        SwerveDriveKinematics.desaturateWheelSpeeds(states, moduleMaxSpeed);
        frontLeft.setDesiredState(states[0]);
        frontRight.setDesiredState(states[1]);
        backLeft.setDesiredState(states[2]);
        backRight.setDesiredState(states[3]);
    }

    public void drive(final SwerveModuleState[] states) {
        drive(states, Constants.Swerve.Modules.MODULE_MAX_SPEED_M_PER_SEC);
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
        return run(() -> drive(
                xSpeedSupplier.getAsDouble(),
                ySpeedSupplier.getAsDouble(),
                rotSupplier.getAsDouble(),
                true
        ));
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
