package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
    @AutoLog
    class ShooterIOInputs {
        double topPositionRots = 0.0;
        double topVelocityRotsPerSec = 0.0;
        double topCurrentAmps = 0.0;
        double topTempCelsius = 0.0;

        double bottomPositionRots = 0.0;
        double bottomVelocityRotsPerSec = 0.0;
        double bottomCurrentAmps = 0.0;
        double bottomTempCelsius = 0.0;
    }

    /**
     * Updates the set of loggable inputs.
     * @param inputs Logged class of IOInputs
     * @see ShooterIOInputs
     * @see AutoLog
     */
    default void updateInputs(final ShooterIOInputs inputs) {}

    default void periodic() {}

    default void config() {}

    /**
     * Called <b>after</b> {@link ShooterIO#config()}, intended for any initialization that needs to happen post-config
     * and cannot happen pre-config (i.e. in the constructor)
     */
    default void initialize() {}

    default void setInputs(final double desiredTopVelocity, final double desiredBottomVelocity) {}

    default void setCharacterizationVolts(final double topVolts, final double bottomVolts) {}

    default void setCharacterizationTorqueCurrent(final double topTorqueCurrentAmps, final double bottomTorqueCurrentAmps) {}
}
