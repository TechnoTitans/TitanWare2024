package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
    @AutoLog
    class IntakeIOInputs {
        public double intakeFrontMotorPositionRots = 0.0;
        public double intakeFrontMotorVelocityRotsPerSec = 0.0;
        public double intakeFrontMotorTorqueCurrentAmps = 0.0;
        public double intakeFrontMotorTempCelsius = 0.0;

        public double intakeBackMotorPositionRots = 0.0;
        public double intakeBackVelocityRotsPerSec = 0.0;
        public double intakeBackTorqueCurrentAmps = 0.0;
        public double intakeBackTempCelsius = 0.0;

        public double shooterFeederMotorPositionRots = 0.0;
        public double shooterFeederMotorVelocityRotsPerSec = 0.0;
        public double shooterFeederMotorTorqueCurrentAmps = 0.0;
        public double shooterFeederMotorTempCelsius = 0.0;
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
            final double frontRollersVelocity,
            final double backRollersVelocity,
            final double shooterFeederRollerVelocity
    ) {}

    default void setCharacterizationTorqueCurrent(
            final double frontRollersTorqueCurrentAmps,
            final double backRollersTorqueCurrentAmps,
            final double shooterFeederRollerTorqueCurrentAmps
    ) {}
}
