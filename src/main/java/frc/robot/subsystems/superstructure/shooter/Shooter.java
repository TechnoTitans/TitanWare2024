package frc.robot.subsystems.superstructure.shooter;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.math.MathUtil;
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

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import static edu.wpi.first.units.Units.*;

public class Shooter extends SubsystemBase {
    protected static final String LogKey = "Shooter";

    private static final double VelocityToleranceRotsPerSec = 2.5;
    private final ShooterIO shooterIO;
    private final ShooterIOInputsAutoLogged inputs;

    private final SysIdRoutine voltageSysIdRoutine;
    private final SysIdRoutine torqueCurrentSysIdRoutine;

    public Trigger atVelocitySetpoint = new Trigger(this::atVelocitySetpoint);

    private Goal goal = Goal.STOP;
    private Goal previousGoal = goal;

    private final VelocitySetpoint setpoint;

    private static class VelocitySetpoint {
        public double ampVelocityRotsPerSec = 0;
        public double leftFlywheelVelocityRotsPerSec = 0;
        public double rightFlywheelVelocityRotsPerSec = 0;
    }

    public enum Goal {
        NONE(0, 0, 0),
        STOP(0, 0, 0),
        IDLE(40, 40, 40),
        EJECT(80, 80, 80),
        BACK_FEED(-60, -60, -60),
        AMP(60, -60, -60),
        SUBWOOFER(80, 80, 80);

        private final double ampVelocity;
        private final double leftFlywheelVelocity;
        private final double rightFlywheelVelocity;

        Goal(
                final double ampVelocity,
                final double leftFlywheelVelocity,
                final double rightFlywheelVelocity
        ) {
            this.ampVelocity = ampVelocity;
            this.leftFlywheelVelocity = leftFlywheelVelocity;
            this.rightFlywheelVelocity = rightFlywheelVelocity;
        }

        public double getAmpVelocity() {
            return ampVelocity;
        }

        public double getLeftFlywheelVelocity() {
            return leftFlywheelVelocity;
        }

        public double getRightFlywheelVelocity() {
            return rightFlywheelVelocity;
        }
    }

    public Shooter(final Constants.RobotMode mode, final HardwareConstants.ShooterConstants shooterConstants) {
        this.shooterIO = switch (mode) {
            case REAL -> new ShooterIOReal(shooterConstants);
            case SIM -> new ShooterIOSim(shooterConstants);
            case REPLAY -> new ShooterIO() {};
        };

        this.inputs = new ShooterIOInputsAutoLogged();
        this.voltageSysIdRoutine = makeVoltageSysIdRoutine(
                Volts.of(2).per(Second),
                Volts.of(10),
                Seconds.of(10)
        );
        this.torqueCurrentSysIdRoutine = makeTorqueCurrentSysIdRoutine(
                Amps.of(4).per(Second),
                Amps.of(40),
                Seconds.of(10)
        );

        this.setpoint = new VelocitySetpoint();
        this.shooterIO.config();
    }

    @Override
    public void periodic() {
        final double shooterPeriodicUpdateStart = Logger.getRealTimestamp();

        shooterIO.updateInputs(inputs);

        Logger.processInputs(LogKey, inputs);
        Logger.recordOutput(
                LogKey + "/PeriodicIOPeriodMs",
                LogUtils.microsecondsToMilliseconds(Logger.getRealTimestamp() - shooterPeriodicUpdateStart)
        );

        if (goal != Goal.NONE && previousGoal != goal) {
            setpoint.ampVelocityRotsPerSec = goal.getAmpVelocity();
            setpoint.leftFlywheelVelocityRotsPerSec = goal.getLeftFlywheelVelocity();
            setpoint.rightFlywheelVelocityRotsPerSec = goal.getRightFlywheelVelocity();
            shooterIO.toVelocity(
                    setpoint.ampVelocityRotsPerSec,
                    setpoint.leftFlywheelVelocityRotsPerSec,
                    setpoint.rightFlywheelVelocityRotsPerSec
            );

            this.previousGoal = goal;
        } else if (goal == Goal.NONE) {
            shooterIO.toVelocity(
                    setpoint.ampVelocityRotsPerSec,
                    setpoint.leftFlywheelVelocityRotsPerSec,
                    setpoint.rightFlywheelVelocityRotsPerSec
            );
            this.previousGoal = Goal.NONE;
        }

        Logger.recordOutput(LogKey + "/Goal", goal.toString());
        Logger.recordOutput(LogKey + "/AtVelocitySetpoint", atVelocitySetpoint());
        Logger.recordOutput(LogKey + "/VelocitySetpoint/AmpVelocityRotsPerSec", setpoint.ampVelocityRotsPerSec);
        Logger.recordOutput(
                LogKey + "/VelocitySetpoint/LeftFlywheelVelocityRotsPerSec",
                setpoint.leftFlywheelVelocityRotsPerSec
        );
        Logger.recordOutput(
                LogKey + "/VelocitySetpoint/RightFlywheelVelocityRotsPerSec",
                setpoint.rightFlywheelVelocityRotsPerSec
        );
    }

    private boolean atVelocitySetpoint() {
        return MathUtil.isNear(
                setpoint.leftFlywheelVelocityRotsPerSec,
                inputs.leftVelocityRotsPerSec,
                VelocityToleranceRotsPerSec
        ) && MathUtil.isNear(
                setpoint.rightFlywheelVelocityRotsPerSec,
                inputs.rightVelocityRotsPerSec,
                VelocityToleranceRotsPerSec
        ) && MathUtil.isNear(
                setpoint.ampVelocityRotsPerSec,
                inputs.ampVelocityRotsPerSec,
                VelocityToleranceRotsPerSec
        );
    }

    public Command toGoal(final Goal goal) {
        return runEnd(() -> this.goal = goal, () -> this.goal = Goal.IDLE);
    }

    public Command toVelocityCommand(
            final DoubleSupplier ampVelocityRotsPerSec,
            final DoubleSupplier leftFlywheelVelocityRotsPerSec,
            final DoubleSupplier rightFlywheelVelocityRotsPerSec
    ) {
        return Commands.sequence(
                Commands.runOnce(() -> this.goal = Goal.NONE),
                run(() -> {
                    setpoint.ampVelocityRotsPerSec = ampVelocityRotsPerSec.getAsDouble();
                    setpoint.leftFlywheelVelocityRotsPerSec = leftFlywheelVelocityRotsPerSec.getAsDouble();
                    setpoint.rightFlywheelVelocityRotsPerSec = rightFlywheelVelocityRotsPerSec.getAsDouble();
                })
        );
    }

    public Command runVoltageCommand(
            final double ampVoltage,
            final double leftFlywheelVoltage,
            final double rightFlywheelVoltage
    ) {
        return run(() -> shooterIO.toVoltage(
                ampVoltage,
                leftFlywheelVoltage,
                rightFlywheelVoltage
        ));
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
                        voltageMeasure -> shooterIO.toVoltage(
                                voltageMeasure.in(Volts),
                                voltageMeasure.in(Volts),
                                voltageMeasure.in(Volts)
                        ),
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
                        // we need to lie to SysId here, because it only takes voltage instead of current
                        Volts.per(Second).of(currentRampRate.baseUnitMagnitude()),
                        Volts.of(stepCurrent.baseUnitMagnitude()),
                        timeout,
                        state -> SignalLogger.writeString(String.format("%s-state", LogKey), state.toString())
                ),
                new SysIdRoutine.Mechanism(
                        voltageMeasure -> shooterIO.toTorqueCurrent(
                                // this is really in amps, not volts
                                voltageMeasure.in(Volts),
                                voltageMeasure.in(Volts),
                                voltageMeasure.in(Volts)
                        ),
                        null,
                        this
                )
        );
    }

    private Command makeSysIdCommand(final SysIdRoutine sysIdRoutine) {
        final BooleanSupplier isStopped = () ->
                MathUtil.isNear(0, inputs.ampVelocityRotsPerSec, 1e-2)
                        && MathUtil.isNear(0, inputs.leftVelocityRotsPerSec, 1e-2)
                        && MathUtil.isNear(0, inputs.rightVelocityRotsPerSec, 1e-2);

        return Commands.sequence(
                sysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward)
                        .withTimeout(10),
                Commands.waitUntil(isStopped),
                sysIdRoutine.quasistatic(SysIdRoutine.Direction.kReverse)
                        .withTimeout(10),
                Commands.waitUntil(isStopped),
                sysIdRoutine.dynamic(SysIdRoutine.Direction.kForward)
                        .withTimeout(6),
                Commands.waitUntil(isStopped),
                sysIdRoutine.dynamic(SysIdRoutine.Direction.kReverse)
                        .withTimeout(6)
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
