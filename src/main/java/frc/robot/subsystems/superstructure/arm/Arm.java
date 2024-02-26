package frc.robot.subsystems.superstructure.arm;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.units.*;
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
    protected static final String LogKey = "Arm";
    private static final double PositionToleranceRots = 0.1;

    private final ArmIO armIO;
    private final ArmIOInputsAutoLogged inputs;

    private final SysIdRoutine voltageSysIdRoutine;
    private final SysIdRoutine torqueCurrentSysIdRoutine;

    private final PositionSetpoint setpoint;
    private final PositionSetpoint pivotSoftLowerLimit;
    private final PositionSetpoint pivotSoftUpperLimit;
    public static class PositionSetpoint {
        public double pivotPositionRots = 0;

        public PositionSetpoint withPivotPositionRots(final double pivotPositionRots) {
            this.pivotPositionRots = pivotPositionRots;
            return this;
        }
    }

    public Arm(final Constants.RobotMode mode, final HardwareConstants.ArmConstants armConstants) {
        this.armIO = switch (mode) {
            case REAL -> new ArmIOReal(armConstants);
            case SIM -> new ArmIOSim(armConstants);
            case REPLAY -> new ArmIO() {};
        };

        this.inputs = new ArmIOInputsAutoLogged();
        this.voltageSysIdRoutine = makeVoltageSysIdRoutine(
                Volts.of(2).per(Second),
                Volts.of(10),
                Seconds.of(4)
        );
        this.torqueCurrentSysIdRoutine = makeTorqueCurrentSysIdRoutine(
                Amps.of(2).per(Second),
                Amps.of(8),
                Seconds.of(4)
        );

        this.setpoint = new PositionSetpoint();
        this.pivotSoftLowerLimit = new PositionSetpoint()
                .withPivotPositionRots(armConstants.pivotSoftLowerLimitRots());
        this.pivotSoftUpperLimit = new PositionSetpoint()
                .withPivotPositionRots(armConstants.pivotSoftUpperLimitRots());

        this.armIO.config(pivotSoftLowerLimit, pivotSoftUpperLimit);
    }

    @Override
    public void periodic() {
        final double armPeriodicUpdateStart = Logger.getRealTimestamp();

        armIO.updateInputs(inputs);

        Logger.processInputs(LogKey, inputs);
        Logger.recordOutput(
                LogKey + "/PeriodicIOPeriodMs",
                LogUtils.microsecondsToMilliseconds(Logger.getRealTimestamp() - armPeriodicUpdateStart)
        );
    }

    public Trigger atPositionSetpointTrigger = new Trigger(this::atPositionSetpoint);
    private boolean atPositionSetpoint() {
        return Math.abs(setpoint.pivotPositionRots - inputs.leftPivotPositionRots) <= PositionToleranceRots
                && Math.abs(setpoint.pivotPositionRots - inputs.rightPivotPositionRots) <= PositionToleranceRots;
    }

    public Trigger atSoftLimitTrigger = new Trigger(this::atSoftLimit);
    private boolean atSoftLimit() {
        return inputs.leftPivotPositionRots >= pivotSoftUpperLimit.pivotPositionRots
                || inputs.leftPivotPositionRots <= pivotSoftLowerLimit.pivotPositionRots;
    }

    public Command toPivotPositionCommand(final double pivotPositionRots) {
        return Commands.sequence(
                runOnce(() -> {
                    setpoint.pivotPositionRots = pivotPositionRots;
                    armIO.toPivotPosition(pivotPositionRots);
                }),
                Commands.waitUntil(atPositionSetpointTrigger)
        );
    }

    public Command toPivotVoltageCommand(final double pivotVoltageVolts) {
        return runOnce(() -> armIO.toPivotVoltage(pivotVoltageVolts));
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
                        voltageMeasure -> armIO.toPivotVoltage(voltageMeasure.in(Volts)),
                        null,
                        this
                )
        );
    }

    // TODO: this needs to respect software limits during the test, should probably make it a routine
    private SysIdRoutine makeTorqueCurrentSysIdRoutine(
            final Measure<Velocity<Current>> currentRampRate,
            final Measure<Current> stepCurrent,
            final Measure<Time> timeout
    ) {
        return new SysIdRoutine(
                new SysIdRoutine.Config(
                        Volts.per(Second).of(currentRampRate.baseUnitMagnitude()),
                        Volts.of(stepCurrent.baseUnitMagnitude()),
                        timeout,
                        state -> SignalLogger.writeString("state", state.toString())
                ),
                new SysIdRoutine.Mechanism(
                        voltageMeasure -> armIO.toPivotTorqueCurrent(voltageMeasure.in(Volts)),
                        null,
                        this
                )
        );
    }

    private Command makeSysIdCommand(final SysIdRoutine sysIdRoutine) {
        return Commands.sequence(
                sysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward)
                        .until(atSoftLimitTrigger),
                Commands.waitSeconds(4),
                sysIdRoutine.quasistatic(SysIdRoutine.Direction.kReverse)
                        .until(atSoftLimitTrigger),
                Commands.waitSeconds(6),
                sysIdRoutine.dynamic(SysIdRoutine.Direction.kForward)
                        .until(atSoftLimitTrigger),
                Commands.waitSeconds(4),
                sysIdRoutine.dynamic(SysIdRoutine.Direction.kReverse)
                        .until(atSoftLimitTrigger)
        );
    }

    @SuppressWarnings("unused")
    public Command voltageSysIdCommand() {
        return makeSysIdCommand(voltageSysIdRoutine);
    }

    @SuppressWarnings("unused")
    public Command torqueCurrentSysIdCommand() {
        return makeSysIdCommand(torqueCurrentSysIdRoutine);
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
