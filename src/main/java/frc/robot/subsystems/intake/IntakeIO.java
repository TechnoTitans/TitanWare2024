package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
    @AutoLog
    class IntakeIOInputs {
        public double rollerMotorPositionRots = 0.0;
        public double rollerMotorVelocityRotsPerSec = 0.0;
        public double rollerMotorTorqueCurrentAmps = 0.0;
        public double rollerMotorTempCelsius = 0.0;

        public double deployMotorPositionRots = 0.0;
        public double deployMotorVelocityRotsPerSec = 0.0;
        public double deployMotorTorqueCurrentAmps = 0.0;
        public double deployMotorTempCelsius = 0.0;
        public String deployMotorLimitSwitch = "Unknown";
    }

    /**
     * Updates the set of loggable inputs.
     * @param inputs Logged class of IOInputs
     * @see IntakeIOInputs
     * @see AutoLog
     */
    default void updateInputs(final IntakeIOInputs inputs) {}

    /**
     * Periodic call to update the elevator,
     * this could include but isn't limited to updating states and motor setpoints
     */
    default void periodic() {}

    /**
     * Config call, should only be called once
     */
    default void config() {}

    /**
     * Called <b>after</b> {@link IntakeIO#config()}, intended for any initialization that needs to happen post-config
     * and cannot happen pre-config (i.e. in the constructor)
     */
    default void initialize() {}

//    default void setInputs() {}
}
