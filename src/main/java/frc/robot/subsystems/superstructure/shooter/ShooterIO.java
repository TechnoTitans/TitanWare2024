package frc.robot.subsystems.superstructure.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
    @AutoLog
    class ShooterIOInputs {
        double ampPositionRots = 0.0;
        double ampVelocityRotsPerSec = 0.0;
        double ampVoltageVolts = 0.0;
        double ampCurrentAmps = 0.0;
        double ampTempCelsius = 0.0;

        double leftPositionRots = 0.0;
        double leftVelocityRotsPerSec = 0.0;
        double leftVoltageVolts = 0.0;
        double leftCurrentAmps = 0.0;
        double leftTempCelsius = 0.0;

        double rightPositionRots = 0.0;
        double rightVelocityRotsPerSec = 0.0;
        double rightVoltageVolts = 0.0;
        double rightCurrentAmps = 0.0;
        double rightTempCelsius = 0.0;
    }

    /**
     * Updates the set of loggable inputs.
     * @param inputs Logged class of IOInputs
     * @see ShooterIOInputs
     * @see AutoLog
     */
    default void updateInputs(final ShooterIOInputs inputs) {}

    default void config() {}

    default void toVelocity(
            final double ampVelocity,
            final double leftFlywheelVelocity,
            final double rightFlywheelVelocity
    ) {}

    default void toVoltage(
            final double ampVolts,
            final double leftVolts,
            final double rightVolts
    ) {}

    default void toTorqueCurrent(
            final double ampTorqueCurrentAmps,
            final double leftTorqueCurrentAmps,
            final double rightTorqueCurrentAmps
    ) {}
}
