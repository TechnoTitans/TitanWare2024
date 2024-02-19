package frc.robot.subsystems.superstructure.arm;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Time;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.constants.Constants;
import frc.robot.constants.HardwareConstants;
import frc.robot.utils.logging.LogUtils;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.*;

public class Arm extends SubsystemBase {
    protected static final String logKey = "Arm";
    private static final double PositionToleranceRots = 0.1;

    private final ArmIO armIO;
    private final ArmIOInputsAutoLogged inputs;

    private final SysIdRoutine voltageSysIdRoutine;

    private final PositionSetpoint setpoint;
    private static class PositionSetpoint {
        public double pivotPositionRots = 0;
    }

    public Arm(final Constants.RobotMode mode, final HardwareConstants.ArmConstants armConstants) {
        // TODO: make real implementation
        this.armIO = switch (mode) {
            case SIM -> new ArmIOSim(armConstants);
            case REAL, REPLAY -> new ArmIO() {};
        };

        this.inputs = new ArmIOInputsAutoLogged();
        this.voltageSysIdRoutine = makeVoltageSysIdRoutine(
                Volts.of(2).per(Second),
                Volts.of(10),
                Seconds.of(4)
        );

        this.setpoint = new PositionSetpoint();
        this.armIO.config();
    }

    @Override
    public void periodic() {
        final double armPeriodicUpdateStart = Logger.getRealTimestamp();

        armIO.updateInputs(inputs);

        Logger.processInputs(logKey, inputs);
        Logger.recordOutput(
                logKey + "/PeriodicIOPeriodMs",
                LogUtils.microsecondsToMilliseconds(Logger.getRealTimestamp() - armPeriodicUpdateStart)
        );
    }

    public Trigger atPosition = new Trigger(this::atPosition);
    private boolean atPosition() {
        return Math.abs(setpoint.pivotPositionRots - inputs.leftPivotPositionRots) <= PositionToleranceRots
                && Math.abs(setpoint.pivotPositionRots - inputs.rightPivotPositionRots) <= PositionToleranceRots;
    }

    public Command toPivotPositionCommand(final double pivotPositionRots) {
        return Commands.sequence(
                runOnce(() -> {
                    setpoint.pivotPositionRots = pivotPositionRots;
                    armIO.toPivotPosition(pivotPositionRots);
                }),
                Commands.waitUntil(atPosition)
        );
    }

    // TODO: this needs to respect software limits during the test, should probably make it a routine
    private SysIdRoutine makeVoltageSysIdRoutine(
            final Measure<Velocity<Voltage>> voltageRampRate,
            final Measure<Voltage> stepVoltage,
            final Measure<Time> timeout
    ) {
        return new SysIdRoutine(
                new SysIdRoutine.Config(
                        voltageRampRate,
                        stepVoltage,
                        timeout,
                        state -> SignalLogger.writeString("state", state.toString())
                ),
                new SysIdRoutine.Mechanism(
                        voltageMeasure -> armIO.setCharacterizationVolts(voltageMeasure.in(Volts)),
                        null,
                        this
                )
        );
    }

    @SuppressWarnings("unused")
    public Command voltageSysIdQuasistaticTestCommand(final SysIdRoutine.Direction direction) {
        return voltageSysIdRoutine.quasistatic(direction);
    }

    @SuppressWarnings("unused")
    public Command voltageSysIdDynamicTestCommand(final SysIdRoutine.Direction direction) {
        return voltageSysIdRoutine.dynamic(direction);
    }
}
