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

import java.util.function.DoubleSupplier;

import static edu.wpi.first.units.Units.*;

public class Shooter extends SubsystemBase {
    protected static final String LogKey = "Shooter";

    private static final double VelocityToleranceRotsPerSec = 0.1;
    private final ShooterIO shooterIO;
    private final ShooterIOInputsAutoLogged inputs;

    private final SysIdRoutine voltageSysIdRoutine;
    private final SysIdRoutine torqueCurrentSysIdRoutine;

    public Trigger atVelocityTrigger = new Trigger(this::atVelocitySetpoint);

    private Goal goal;
    private final VelocitySetpoint setpoint;

    private static class VelocitySetpoint {
        public double ampVelocityRotsPerSec = 0;
        public double leftFlywheelVelocityRotsPerSec = 0;
        public double rightFlywheelVelocityRotsPerSec = 0;
    }

    public enum Goal {
        STOP(() -> 0, () -> 0, () -> 0),
        IDLE(() -> 10, () -> 10, () -> 10),
        AMP(() -> 4, () -> 4, () -> 4),
        SUBWOOFER(() -> 6, () -> 6, () -> 6),
        AIM_SPEAKER(() -> 0, () -> 0, () -> 0);

        private final DoubleSupplier ampVelocitySupplier;
        private final DoubleSupplier leftFlywheelVelocitySupplier;
        private final DoubleSupplier rightFlywheelVelocitySupplier;

        Goal(
                final DoubleSupplier ampVelocitySupplier,
                final DoubleSupplier leftFlywheelVelocitySupplier,
                final DoubleSupplier rightFlywheelVelocitySupplier
        ) {
            this.ampVelocitySupplier = ampVelocitySupplier;
            this.leftFlywheelVelocitySupplier = leftFlywheelVelocitySupplier;
            this.rightFlywheelVelocitySupplier = rightFlywheelVelocitySupplier;
        }

        public double getAmpVelocity() {
            return ampVelocitySupplier.getAsDouble();
        }

        public double getLeftFlywheelVelocity() {
            return leftFlywheelVelocitySupplier.getAsDouble();
        }

        public double getRightFlywheelVelocity() {
            return rightFlywheelVelocitySupplier.getAsDouble();
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

        final double previousAmpVelocity = setpoint.ampVelocityRotsPerSec;
        final double previousLeftFlywheelVelocity = setpoint.leftFlywheelVelocityRotsPerSec;
        final double previousRightFlywheelVelocity = setpoint.rightFlywheelVelocityRotsPerSec;

        setpoint.ampVelocityRotsPerSec = goal.getAmpVelocity();
        setpoint.leftFlywheelVelocityRotsPerSec = goal.getLeftFlywheelVelocity();
        setpoint.rightFlywheelVelocityRotsPerSec = goal.getRightFlywheelVelocity();

        if (setpoint.ampVelocityRotsPerSec != previousAmpVelocity
                || setpoint.leftFlywheelVelocityRotsPerSec != previousLeftFlywheelVelocity
                || setpoint.rightFlywheelVelocityRotsPerSec != previousRightFlywheelVelocity
        ) {
            shooterIO.toVelocity(
                    setpoint.ampVelocityRotsPerSec,
                    setpoint.leftFlywheelVelocityRotsPerSec,
                    setpoint.rightFlywheelVelocityRotsPerSec
            );
        }

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
        ) || MathUtil.isNear(
                setpoint.rightFlywheelVelocityRotsPerSec,
                inputs.rightVelocityRotsPerSec,
                VelocityToleranceRotsPerSec
        );
    }

    public Command toGoal(final Goal goal) {
        return runOnce(() -> this.goal = goal);
    }

    public Command toVelocityCommand(
            final double ampVelocityRotsPerSec,
            final double leftFlywheelVelocityRotsPerSec,
            final double rightFlywheelVelocityRotsPerSec
    ) {
        return Commands.sequence(
                runOnce(() -> {
                    setpoint.ampVelocityRotsPerSec = ampVelocityRotsPerSec;
                    setpoint.leftFlywheelVelocityRotsPerSec = leftFlywheelVelocityRotsPerSec;
                    setpoint.rightFlywheelVelocityRotsPerSec = rightFlywheelVelocityRotsPerSec;

                    shooterIO.toVelocity(ampVelocityRotsPerSec, leftFlywheelVelocityRotsPerSec, rightFlywheelVelocityRotsPerSec);
                }),
                Commands.waitUntil(atVelocityTrigger)
        );
    }

    public Command toVoltageCommand(
            final double ampVoltage,
            final double leftFlywheelVoltage,
            final double rightFlywheelVoltage
    ) {
        return runOnce(() -> shooterIO.toVoltage(ampVoltage, leftFlywheelVoltage, rightFlywheelVoltage));
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
        return Commands.sequence(
                sysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward)
                        .withTimeout(10),
                Commands.waitSeconds(4),
                sysIdRoutine.quasistatic(SysIdRoutine.Direction.kReverse)
                        .withTimeout(10),
                Commands.waitSeconds(6),
                sysIdRoutine.dynamic(SysIdRoutine.Direction.kForward)
                        .withTimeout(6),
                Commands.waitSeconds(4),
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
