package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
    @AutoLog
    class IntakeIOInputs {
        // TODO @max: fix the names here, "Motor" is often redundant or completely incorrect
        public double rightMotorPositionRots = 0.0;
        public double rightMotorVelocityRotsPerSec = 0.0;
        public double rightMotorVoltage = 0.0;
        public double rightMotorTorqueCurrentAmps = 0.0;
        public double rightMotorTempCelsius = 0.0;

        public double leftMotorPositionRots = 0.0;
        public double leftVelocityRotsPerSec = 0.0;
        public double leftMotorVoltage = 0.0;
        public double leftTorqueCurrentAmps = 0.0;
        public double leftTempCelsius = 0.0;

        public double shooterFeederMotorPositionRots = 0.0;
        public double shooterFeederMotorVelocityRotsPerSec = 0.0;
        public double shooterFeederMotorVoltage = 0.0;
        public double shooterFeederMotorTorqueCurrentAmps = 0.0;
        public double shooterFeederMotorTempCelsius = 0.0;

        public boolean gamePieceDetected = false;
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
