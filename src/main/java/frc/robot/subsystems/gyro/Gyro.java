package frc.robot.subsystems.gyro;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import frc.robot.constants.Constants;
import frc.robot.constants.HardwareConstants;
import frc.robot.subsystems.drive.Swerve;
import frc.robot.subsystems.drive.SwerveModule;
import frc.robot.utils.logging.LogUtils;
import org.littletonrobotics.junction.Logger;

public class Gyro {
    protected static final String logKey = "Gyro";

    private final GyroIO gyroIO;

    private final HardwareConstants.GyroConstants gyroConstants;
    private final GyroIOInputsAutoLogged inputs;
    private final boolean isReal;

    public Gyro(
            final Constants.RobotMode mode,
            final HardwareConstants.GyroConstants gyroConstants,
            final Swerve.OdometryThreadRunner odometryThreadRunner,
            final SwerveDriveKinematics kinematics,
            final SwerveModule[] swerveModules
    ) {
        this.gyroConstants = gyroConstants;
        this.gyroIO = switch (mode) {
            case REAL -> new GyroIOPigeon2(gyroConstants, odometryThreadRunner);
            case SIM -> new GyroIOSim(gyroConstants, odometryThreadRunner, kinematics, swerveModules);
            case REPLAY -> new GyroIO() {};
        };

        this.inputs = new GyroIOInputsAutoLogged();
        this.isReal = Constants.CURRENT_MODE == Constants.RobotMode.REAL;

        this.gyroIO.config();
    }

    public void periodic() {
        final double gyroPeriodicUpdateStart = Logger.getRealTimestamp();

        gyroIO.periodic();
        gyroIO.updateInputs(inputs);
        Logger.processInputs(logKey, inputs);

        Logger.recordOutput(
                logKey + "/PeriodicIOPeriodMs",
                LogUtils.microsecondsToMilliseconds(Logger.getRealTimestamp() - gyroPeriodicUpdateStart)
        );
    }

    public boolean hasHardwareFault() {
        return inputs.hasHardwareFault;
    }

    /**
     * Get the gyro constants
     * @return the {@link frc.robot.constants.HardwareConstants.GyroConstants}
     * @see frc.robot.constants.HardwareConstants.GyroConstants
     */
    public HardwareConstants.GyroConstants getGyroConstants() {
        return gyroConstants;
    }

    /**
     * Get whether this Gyro is real (true/real if hardware exists, false if hardware does not exist - i.e. in a sim)
     * @return true if the Gyro is real, false if not
     */
    public boolean isReal() {
        return isReal;
    }

    /**
     * Get the current yaw (heading) reported by the Gyro
     * @return the current yaw (deg)
     */
    public double getYaw() {
        return inputs.yawPositionDeg;
    }

    /**
     * Get the current yaw (heading) as a {@link Rotation2d}
     * @return the {@link Rotation2d} of the current yaw
     * @see Rotation2d
     */
    public Rotation2d getYawRotation2d() {
        return Rotation2d.fromDegrees(getYaw());
    }

    /**
     * Get the current yaw (heading) velocity (AngularVelocityZ) reported by the Gyro
     * @return the current yaw velocity (deg/sec)
     */
    public double getYawVelocity() { return inputs.yawVelocityDegPerSec; }

    public double[] getOdometryTimestamps() {
        return inputs.odometryTimestampsSec;
    }

    public double[] getOdometryYawPositions() {
        return inputs.odometryYawPositionsDeg;
    }

    /**
     * Get the current yaw (heading) velocity (AngularVelocityZ) as a {@link Rotation2d}
     * @return the {@link Rotation2d} of the current yaw velocity (deg/sec)
     * @see Rotation2d
     */
    public Rotation2d getYawVelocityRotation2d() { return Rotation2d.fromDegrees(getYawVelocity()); }

    /**
     * Get the current pitch reported by the Gyro
     * @return the current pitch (deg)
     */
    public double getPitch() {
        return inputs.pitchPositionDeg;
    }

    /**
     * Get the current pitch as a {@link Rotation2d}
     * @return the {@link Rotation2d} of the current pitch
     * @see Rotation2d
     */
    public Rotation2d getPitchRotation2d() {
        return Rotation2d.fromDegrees(getPitch());
    }

    public double getPitchVelocity() {
        return inputs.pitchVelocityDegPerSec;
    }

    /**
     * Get the current roll reported by the Gyro
     * @return the current roll (deg)
     */
    public double getRoll() {
        return inputs.rollPositionDeg;
    }

    /**
     * Get the current pitch as a {@link Rotation2d}
     * @return the {@link Rotation2d} of the current pitch
     * @see Rotation2d
     */
    public Rotation2d getRollRotation2d() {
        return Rotation2d.fromDegrees(getRoll());
    }

    public double getRollVelocity() {
        return inputs.rollVelocityDegPerSec;
    }

    /**
     * See {@link GyroIO#setAngle(Rotation2d)}
     */
    public void setAngle(final Rotation2d angle) {
        gyroIO.setAngle(angle);
    }

    /**
     * See {@link GyroIO#zeroRotation()}
     */
    public void zeroRotation() {
        gyroIO.zeroRotation();
    }
}
