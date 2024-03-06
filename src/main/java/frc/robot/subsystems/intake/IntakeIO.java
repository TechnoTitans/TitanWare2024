package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
    @AutoLog
    class IntakeIOInputs {
        public double rightPositionRots = 0.0;
        public double rightVelocityRotsPerSec = 0.0;
        public double rightVoltage = 0.0;
        public double rightTorqueCurrentAmps = 0.0;
        public double rightTempCelsius = 0.0;

        public double leftPositionRots = 0.0;
        public double leftVelocityRotsPerSec = 0.0;
        public double leftVoltage = 0.0;
        public double leftTorqueCurrentAmps = 0.0;
        public double leftTempCelsius = 0.0;

        public double shooterFeederPositionRots = 0.0;
        public double shooterFeederVelocityRotsPerSec = 0.0;
        public double shooterFeederVoltage = 0.0;
        public double shooterFeederTorqueCurrentAmps = 0.0;
        public double shooterFeederTempCelsius = 0.0;

        public boolean shooterBeamBreak = false;
    }

    /**
     * Updates the set of loggable inputs.
     * @param inputs Logged class of IOInputs
     * @see IntakeIOInputs
     * @see AutoLog
     */
    default void updateInputs(final IntakeIOInputs inputs) {}

    /**
     * Config call, should only be called once
     */
    default void config() {}

    /**
     * Called <b>after</b> {@link IntakeIO#config()}, intended for any initialization that needs to happen post-config
     * and cannot happen pre-config (i.e. in the constructor)
     */
    default void initialize() {}

    default void toVelocity(
            final double rightRollerVelocity,
            final double leftRollerVelocity,
            final double shooterFeederRollerVelocity
    ) {}

    default void toTorqueCurrent(
            final double rightRollerTorqueCurrentAmps,
            final double leftRollerTorqueCurrentAmps,
            final double shooterFeederRollerTorqueCurrentAmps
    ) {}

    default void toVoltage(
            final double rightRollersVolts,
            final double leftRollersVolts,
            final double shooterFeederRollerVolts
    ) {}
}
