package frc.robot.subsystems.superstructure.shooter;

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

public class Shooter extends SubsystemBase {
    protected static final String logKey = "Shooter";

    private static final double VelocityToleranceRotsPerSec = 0.1;
    private final ShooterIO shooterIO;
    private final ShooterIOInputsAutoLogged inputs;

    private final SysIdRoutine voltageSysIdRoutine;
    private final SysIdRoutine torqueCurrentSysIdRoutine;

    private final VelocitySetpoint setpoint;
    private static class VelocitySetpoint {
        public double leftFlywheelVelocityRotsPerSec = 0;
        public double rightFlywheelVelocityRotsPerSec = 0;
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
                Amps.of(2).per(Second),
                Amps.of(10),
                Seconds.of(10)
        );

        this.setpoint = new VelocitySetpoint();
        this.shooterIO.config();
    }

    @Override
    public void periodic() {
        final double shooterPeriodicUpdateStart = Logger.getRealTimestamp();

        shooterIO.periodic();
        shooterIO.updateInputs(inputs);

        Logger.processInputs(logKey, inputs);
        Logger.recordOutput(
                logKey + "/PeriodicIOPeriodMs",
                LogUtils.microsecondsToMilliseconds(Logger.getRealTimestamp() - shooterPeriodicUpdateStart)
        );
    }

    public Trigger atVelocity = new Trigger(this::atVelocity);
    private boolean atVelocity() {
        return Math.abs(setpoint.leftFlywheelVelocityRotsPerSec - inputs.leftVelocityRotsPerSec) <= VelocityToleranceRotsPerSec
                && Math.abs(setpoint.rightFlywheelVelocityRotsPerSec - inputs.rightVelocityRotsPerSec) <= VelocityToleranceRotsPerSec;
    }

    public Command toVelocityCommand(
            final double leftFlywheelVelocityRotsPerSec,
            final double rightFlywheelVelocityRotsPerSec
    ) {
        return Commands.sequence(
                runOnce(() -> {
                    setpoint.leftFlywheelVelocityRotsPerSec = leftFlywheelVelocityRotsPerSec;
                    setpoint.rightFlywheelVelocityRotsPerSec = rightFlywheelVelocityRotsPerSec;

                    shooterIO.toVelocity(leftFlywheelVelocityRotsPerSec, rightFlywheelVelocityRotsPerSec);
                }),
                Commands.waitUntil(atVelocity)
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
                        state -> SignalLogger.writeString("state", state.toString())
                ),
                new SysIdRoutine.Mechanism(
                        voltageMeasure -> shooterIO.setCharacterizationVolts(
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
                        state -> SignalLogger.writeString("state", state.toString())
                ),
                new SysIdRoutine.Mechanism(
                        voltageMeasure -> shooterIO.setCharacterizationTorqueCurrent(
                                // this is really in amps, not volts
                                voltageMeasure.in(Volts),
                                voltageMeasure.in(Volts)
                        ),
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

    @SuppressWarnings("unused")
    public Command torqueCurrentSysIdQuasistaticTestCommand(final SysIdRoutine.Direction direction) {
        return torqueCurrentSysIdRoutine.quasistatic(direction);
    }

    @SuppressWarnings("unused")
    public Command torqueCurrentSysIdDynamicTestCommand(final SysIdRoutine.Direction direction) {
        return torqueCurrentSysIdRoutine.dynamic(direction);
    }
}
