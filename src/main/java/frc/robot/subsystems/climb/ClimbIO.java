package frc.robot.subsystems.climb;

import org.littletonrobotics.junction.AutoLog;

public interface ClimbIO {
    @AutoLog
    class ClimbIOInputs {
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
    }

    /**
     * Updates the set of loggable inputs.
     * @param inputs Logged class of IOInputs
     * @see ClimbIO.ClimbIOInputs
     * @see AutoLog
     */
    default void updateInputs(final ClimbIO.ClimbIOInputs inputs) {}

    /**
     * Config call, should only be called once
     */
    default void config() {}

    /**
     * Called <b>after</b> {@link ClimbIO#config()}, intended for any initialization that needs to happen post-config
     * and cannot happen pre-config (i.e. in the constructor)
     */
    default void initialize() {}

    default void toPosition(
            final double rightArmPosition,
            final double leftArmPosition
    ) {}

    default void toTorqueCurrent(
            final double rightArmTorqueCurrentAmps,
            final double leftArmTorqueCurrentAmps
    ) {}

    default void toVoltage(
            final double rightArmVolts,
            final double leftArmVolts
    ) {}

}
