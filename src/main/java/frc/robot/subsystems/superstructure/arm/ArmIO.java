package frc.robot.subsystems.superstructure.arm;

import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {
    @AutoLog
    class ArmIOInputs {
        boolean pivotUpperLimitSwitch = false;

        double leftPivotPositionRots = 0;
        double leftPivotVelocityRotsPerSec = 0;
        double leftPivotVoltageVolts = 0;
        double leftPivotTorqueCurrentAmps = 0;
        double leftPivotTempCelsius = 0;

        double rightPivotPositionRots = 0;
        double rightPivotVelocityRotsPerSec = 0;
        double rightPivotVoltageVolts = 0;
        double rightPivotTorqueCurrentAmps = 0;
        double rightPivotTempCelsius = 0;
    }

    /**
     * Updates the set of loggable inputs.
     * @param inputs Logged class of IOInputs
     * @see ArmIOInputs
     * @see AutoLog
     */
    default void updateInputs(final ArmIOInputs inputs) {}

    default void config() {}

    default void configureSoftLimits(
            final Arm.PositionSetpoint pivotSoftLowerLimit,
            final Arm.PositionSetpoint pivotSoftUpperLimit
    ) {}

    default void setPivotPosition(final double pivotPositionRots) {}

    default void toPivotPosition(final double pivotPositionRots) {}

    default void toPivotVoltage(final double pivotVolts) {}

    default void toPivotTorqueCurrent(final double pivotTorqueCurrentAmps) {}
}
