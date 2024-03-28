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
    private static final double PositionToleranceRots = 0.005;
    private static final double VelocityToleranceRotsPerSec = 0.02;

    private final HardwareConstants.ArmConstants armConstants;

    private final ArmIO armIO;
    private final ArmIOInputsAutoLogged inputs;

    private final SysIdRoutine voltageSysIdRoutine;
    private final SysIdRoutine torqueCurrentSysIdRoutine;

    private Goal goal = Goal.STOW;
    private Goal previousGoal = goal;

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

        public boolean atSetpoint(final double pivotPositionRots, final double pivotVelocityRotsPerSec) {
            return MathUtil.isNear(this.pivotPositionRots, pivotPositionRots, PositionToleranceRots)
                    && MathUtil.isNear(0, pivotVelocityRotsPerSec, VelocityToleranceRotsPerSec);
        }
    }

    public enum Goal {
        NONE(0),
        STOW(Units.degreesToRotations(10)),
        AMP(Units.degreesToRotations(95)),
        SUBWOOFER(Units.degreesToRotations(56.5));

        private final double pivotPositionGoal;
        Goal(final double pivotPositionGoal) {
            this.pivotPositionGoal = pivotPositionGoal;
        }

        public double getPivotPositionGoal() {
            return pivotPositionGoal;
        }
    }

    public Arm(final Constants.RobotMode mode, final HardwareConstants.ArmConstants armConstants) {
        this.armConstants = armConstants;
        this.armIO = switch (mode) {
            case REAL -> new ArmIOReal(armConstants);
            case SIM -> new ArmIOSim(armConstants);
            case REPLAY -> new ArmIO() {};
        };

        this.inputs = new ArmIOInputsAutoLogged();
        this.voltageSysIdRoutine = makeVoltageSysIdRoutine(
                Volts.of(4).per(Second),
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

        if (goal != Goal.NONE && previousGoal != goal) {
            setpoint.pivotPositionRots = goal.getPivotPositionGoal();
            armIO.toPivotPosition(setpoint.pivotPositionRots);

            this.previousGoal = goal;
        } else if (goal == Goal.NONE) {
            armIO.toPivotPosition(setpoint.pivotPositionRots);
            this.previousGoal = Goal.NONE;
        }

        Logger.recordOutput(LogKey + "/Goal", goal.toString());
        Logger.recordOutput(LogKey + "/PositionSetpoint/PivotPositionRots", setpoint.pivotPositionRots);
        Logger.recordOutput(LogKey + "/AtPositionSetpoint", atPositionSetpoint());
        Logger.recordOutput(LogKey + "/AtPivotLowerLimit", atPivotLowerLimit());
        Logger.recordOutput(LogKey + "/AtPivotUpperLimit", atPivotUpperLimit());
    }

    private boolean atPositionSetpoint() {
        return setpoint.atSetpoint(inputs.pivotEncoderPositionRots, inputs.pivotEncoderVelocityRotsPerSec);
    }

    private boolean atPivotLowerLimit() {
        return inputs.pivotEncoderPositionRots <= pivotSoftLowerLimit.pivotPositionRots;
    }

    private boolean atPivotUpperLimit() {
        return inputs.pivotEncoderPositionRots >= pivotSoftUpperLimit.pivotPositionRots;
    }

    public Command toGoal(final Goal goal) {
        // TODO: need to standardize on using runOnce vs. runEnd, i.e. whether this command,
        //  on end/interrupt should schedule the default/idle goal (in this case, STOW)
        return runEnd(() -> this.goal = goal, () -> this.goal = Goal.STOW);
    }

    public Command toPivotPositionCommand(final DoubleSupplier pivotPositionRots) {
        return Commands.sequence(
                Commands.runOnce(() -> this.goal = Goal.NONE),
                run(() -> setpoint.pivotPositionRots = pivotPositionRots.getAsDouble())
        );
    }

    public Command runPivotVoltageCommand(final double pivotVoltageVolts) {
        return run(() -> armIO.toPivotVoltage(pivotVoltageVolts));
    }

    @SuppressWarnings("unused")
    public Command homePivotWithCurrentCommand() {
        return Commands.sequence(
                startEnd(
                        () -> armIO.toPivotVoltage(-1),
                        () -> armIO.toPivotVoltage(0)
                ).until(() -> inputs.leftPivotTorqueCurrentAmps >= 25 || inputs.rightPivotTorqueCurrentAmps >= 25),
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
                Commands.waitSeconds(4),
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
