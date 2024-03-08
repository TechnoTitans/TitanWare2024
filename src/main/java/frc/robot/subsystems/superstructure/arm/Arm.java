package frc.robot.subsystems.superstructure.arm;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
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

import java.util.function.DoubleSupplier;

import static edu.wpi.first.units.Units.*;

public class Arm extends SubsystemBase {
    protected static final String LogKey = "Arm";
    private static final double PositionToleranceRots = 0.01;

    private final ArmIO armIO;
    private final ArmIOInputsAutoLogged inputs;

    private final SysIdRoutine voltageSysIdRoutine;
    private final SysIdRoutine torqueCurrentSysIdRoutine;

    private Goal goal = Goal.STOW;
    private final PositionSetpoint setpoint;
    private final PositionSetpoint pivotSoftLowerLimit;
    private final PositionSetpoint pivotSoftUpperLimit;

    public Trigger atPivotSetpoint = new Trigger(this::atPositionSetpoint);
    public Trigger atPivotLowerLimit = new Trigger(this::atPivotLowerLimit);
    public Trigger atPivotUpperLimit = new Trigger(this::atPivotUpperLimit);
    public Trigger atPivotLimit = atPivotLowerLimit.or(atPivotUpperLimit);

    public static class PositionSetpoint {
        public double pivotPositionRots = 0;

        public PositionSetpoint withPivotPositionRots(final double pivotPositionRots) {
            this.pivotPositionRots = pivotPositionRots;
            return this;
        }

        public boolean atSetpoint(final double pivotPositionRots) {
            return MathUtil.isNear(this.pivotPositionRots, pivotPositionRots, PositionToleranceRots);
        }
    }

    public enum Goal {
        ZERO(0),
        STOW(Units.degreesToRotations(10)),
        AMP(Units.degreesToRotations(90)),
        SUBWOOFER(Units.degreesToRotations(55));

        private final double pivotPositionGoal;
        Goal(final double pivotPositionGoal) {
            this.pivotPositionGoal = pivotPositionGoal;
        }

        public double getPivotPositionGoal() {
            return pivotPositionGoal;
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
                Seconds.of(6)
        );
        this.torqueCurrentSysIdRoutine = makeTorqueCurrentSysIdRoutine(
                Amps.of(2).per(Second),
                Amps.of(8),
                Seconds.of(6)
        );

        this.setpoint = new PositionSetpoint();
        this.pivotSoftLowerLimit = new PositionSetpoint()
                .withPivotPositionRots(armConstants.pivotSoftLowerLimitRots());
        this.pivotSoftUpperLimit = new PositionSetpoint()
                .withPivotPositionRots(armConstants.pivotSoftUpperLimitRots());

        this.armIO.config();
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

        final double previousPivotPosition = setpoint.pivotPositionRots;
        setpoint.pivotPositionRots = goal.getPivotPositionGoal();
        if (setpoint.pivotPositionRots != previousPivotPosition) {
            armIO.toPivotPosition(setpoint.pivotPositionRots);
        }

        Logger.recordOutput(LogKey + "/Goal", goal.toString());
        Logger.recordOutput(LogKey + "/PositionSetpoint/PivotPositionRots", setpoint.pivotPositionRots);
        Logger.recordOutput(LogKey + "/AtPositionSetpoint", atPositionSetpoint());
        Logger.recordOutput(LogKey + "/AtPivotLowerLimit", atPivotLowerLimit());
        Logger.recordOutput(LogKey + "/AtPivotUpperLimit", atPivotUpperLimit());
    }

    private boolean atPositionSetpoint() {
        return setpoint.atSetpoint(inputs.leftPivotPositionRots)
                && setpoint.atSetpoint(inputs.rightPivotPositionRots);
    }

    private boolean atPivotLowerLimit() {
        return inputs.pivotLowerLimitSwitch
                || inputs.leftPivotPositionRots <= pivotSoftLowerLimit.pivotPositionRots
                || inputs.rightPivotPositionRots <= pivotSoftLowerLimit.pivotPositionRots;
    }

    private boolean atPivotUpperLimit() {
        return inputs.leftPivotPositionRots >= pivotSoftUpperLimit.pivotPositionRots
                || inputs.rightPivotPositionRots >= pivotSoftUpperLimit.pivotPositionRots;
    }

    public Command toGoal(final Goal goal) {
        return startEnd(() -> this.goal = goal, () -> this.goal = Goal.STOW);
    }

    public Command toPivotPositionCommand(final DoubleSupplier pivotPositionRots) {
        return run(() -> {
            setpoint.pivotPositionRots = pivotPositionRots.getAsDouble();
            armIO.toPivotPosition(setpoint.pivotPositionRots);
        });
    }

    public Command toPivotVoltageCommand(final DoubleSupplier pivotVoltageVolts) {
        return run(() -> armIO.toPivotVoltage(pivotVoltageVolts.getAsDouble()));
    }

    public Command homePivotCommand() {
        // TODO: this could probably be improved to be more robust,
        //  and potentially do a 2nd, slower, pass to be more accurate
        return Commands.sequence(
                startEnd(
                        () -> armIO.toPivotVoltage(-3),
                        () -> armIO.toPivotVoltage(0)
                ).until(() -> inputs.pivotLowerLimitSwitch),
                runOnce(() -> {
                    armIO.setPivotPosition(0);
                    armIO.configureSoftLimits(pivotSoftLowerLimit, pivotSoftUpperLimit);
                })
        );
    }

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
                        state -> SignalLogger.writeString(String.format("%s-state", LogKey), state.toString())
                ),
                new SysIdRoutine.Mechanism(
                        voltageMeasure -> armIO.toPivotVoltage(voltageMeasure.in(Volts)),
                        null,
                        this
                )
        );
    }

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
                        state -> SignalLogger.writeString(String.format("%s-state", LogKey), state.toString())
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
                        .until(atPivotUpperLimit),
                Commands.waitSeconds(4),
                sysIdRoutine.quasistatic(SysIdRoutine.Direction.kReverse)
                        .until(atPivotLowerLimit),
                Commands.waitSeconds(6),
                sysIdRoutine.dynamic(SysIdRoutine.Direction.kForward)
                        .until(atPivotUpperLimit),
                Commands.waitSeconds(4),
                sysIdRoutine.dynamic(SysIdRoutine.Direction.kReverse)
                        .until(atPivotLowerLimit)
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
}
