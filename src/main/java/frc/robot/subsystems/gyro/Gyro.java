package frc.robot.subsystems.gyro;

import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.constants.Constants;
import frc.robot.utils.logging.LogUtils;
import org.littletonrobotics.junction.Logger;

public class Gyro {
    protected static final String logKey = "Gyro";

    private final GyroIO gyroIO;
    private final Pigeon2 pigeon2;
    private final GyroIOInputsAutoLogged inputs;
    private final boolean isReal;

    private final LinearFilter pitchVelocityFilter;

    public Gyro(final GyroIO gyroIO, final Pigeon2 pigeon2) {
        this.gyroIO = gyroIO;
        this.pigeon2 = pigeon2;
        this.inputs = new GyroIOInputsAutoLogged();
        this.isReal = Constants.CURRENT_MODE == Constants.RobotMode.REAL;

        this.pitchVelocityFilter = LinearFilter.movingAverage(16);

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

        Logger.recordOutput("FilteredPitchVelocityDPS", getFilteredPitchVelocity());
    }

    /**
     * Get the underlying Pigeon object from CTRE (no guarantees are made about real/sim)
     * @return the {@link Pigeon2}
     * @see Pigeon2
     */
    public Pigeon2 getPigeon() { return pigeon2; }

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

    public double getFilteredPitchVelocity() {
        return pitchVelocityFilter.calculate(getPitchVelocity());
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
