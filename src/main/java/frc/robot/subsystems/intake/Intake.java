package frc.robot.subsystems.intake;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.units.Current;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Time;
import edu.wpi.first.units.Velocity;
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

public class Intake extends SubsystemBase {
    protected static final String LogKey = "Intake";

    private final IntakeIO intakeIO;
    private final IntakeIOInputsAutoLogged inputs;

    private final VelocitySetpoint setpoint;
    public final Trigger shooterBeamBreakBroken;

    private boolean intakingActive = false;
    public final Trigger intaking;

    private boolean feedingActive = false;
    public final Trigger feeding;

    private static class VelocitySetpoint {
        public double rightRollerVelocityRotsPerSec;
        public double leftRollerVelocityRotsPerSec;
        public double shooterFeederRotsPerSec;
    }

    private final SysIdRoutine torqueCurrentSysIdRoutine;

    public Intake(
            final Constants.RobotMode robotMode,
            final HardwareConstants.IntakeConstants intakeConstants
    ) {
        this.intakeIO = switch (robotMode) {
            case REAL -> new IntakeIOReal(intakeConstants);
            case SIM -> new IntakeIOSim(intakeConstants);
            case REPLAY -> new IntakeIO() {};
        };

        this.inputs = new IntakeIOInputsAutoLogged();

        this.setpoint = new VelocitySetpoint();
        this.shooterBeamBreakBroken = new Trigger(() -> inputs.shooterBeamBreak);

        this.intaking = new Trigger(() -> this.intakingActive);
        this.feeding = new Trigger(() -> this.feedingActive);

        this.torqueCurrentSysIdRoutine = makeTorqueCurrentSysIdRoutine(
                Amps.of(2).per(Second),
                Amps.of(10),
                Seconds.of(10)
        );

        this.intakeIO.config();
        this.intakeIO.initialize();
    }

    @Override
    public void periodic() {
        final double intakeIOPeriodicStart = Logger.getRealTimestamp();
        intakeIO.updateInputs(inputs);
        Logger.processInputs(LogKey, inputs);

        Logger.recordOutput(
                LogKey + "/PeriodicIOPeriodMs",
                LogUtils.microsecondsToMilliseconds(Logger.getRealTimestamp() - intakeIOPeriodicStart)
        );

        Logger.recordOutput(LogKey + "/Intaking", intaking.getAsBoolean());
        Logger.recordOutput(LogKey + "/Feeding", feeding.getAsBoolean());

        Logger.recordOutput(LogKey + "/Setpoint/RightRollerVelocityRotsPerSec", setpoint.rightRollerVelocityRotsPerSec);
        Logger.recordOutput(LogKey + "/Setpoint/LeftRollerVelocityRotsPerSec", setpoint.leftRollerVelocityRotsPerSec);
        Logger.recordOutput(LogKey + "/Setpoint/ShooterFeederRotsPerSec", setpoint.shooterFeederRotsPerSec);
    }

    public Command storeCommand() {
        return runVelocityCommand(4, 4, 4)
                .until(shooterBeamBreakBroken)
                .unless(shooterBeamBreakBroken)
                .andThen(
                        Commands.deadline(
                                Commands.waitUntil(shooterBeamBreakBroken.negate()),
                                runVelocityCommand(-4, -4, -4)
                        )
                )
                .andThen(instantStopCommand());
    }

    public Command intakeCommand(final Command runOnHasNote) {
        return Commands.sequence(
                runOnce(() -> this.intakingActive = true),
                runVelocityCommand(22, 22, 22)
                        .until(shooterBeamBreakBroken),
                instantStopCommand(),
                storeCommand(),
                runOnHasNote.asProxy()
        ).finallyDo(() -> this.intakingActive = false);
    }

    public Command intakeCommand() {
        return intakeCommand(Commands.none());
    }

    public Command feedHalfCommand() {
        return storeCommand()
                .onlyIf(shooterBeamBreakBroken)
                .andThen(Commands.deadline(
                        Commands.waitUntil(shooterBeamBreakBroken),
                        runVelocityCommand(6, 6, 6)
                ));
    }

    public Command feedCommand() {
        return storeCommand()
                .onlyIf(shooterBeamBreakBroken)
                .andThen(Commands.deadline(
                        Commands.waitUntil(shooterBeamBreakBroken)
                                .andThen(Commands.waitUntil(shooterBeamBreakBroken.negate())),
                        runVelocityCommand(18, 18, 18)
                ));
    }

    public Command runEjectOutCommand() {
        return runVoltageCommand(-12, -12, -12);
    }

    public Command runEjectInCommand() {
        return runVoltageCommand(12, 12, 12);
    }

    public Command instantStopCommand() {
        return runOnce(() -> intakeIO.toVoltage(
                0,
                0,
                0
        ));
    }

    public Command runStopCommand() {
        return runVoltageCommand(0, 0, 0);
    }

    public Command runVelocityCommand(
            final double rightRollerVelocityRotsPerSec,
            final double leftRollerVelocityRotsPerSec,
            final double shooterFeederRotsPerSec
    ) {
        return runEnd(() -> {
            setpoint.rightRollerVelocityRotsPerSec = rightRollerVelocityRotsPerSec;
            setpoint.leftRollerVelocityRotsPerSec = leftRollerVelocityRotsPerSec;
            setpoint.shooterFeederRotsPerSec = shooterFeederRotsPerSec;

            intakeIO.toVelocity(
                    rightRollerVelocityRotsPerSec,
                    leftRollerVelocityRotsPerSec,
                    shooterFeederRotsPerSec
            );
        }, () -> intakeIO.toVelocity(0, 0, 0));
    }

    public Command runVoltageCommand(
            final double rightRollerVoltage,
            final double leftRollerVoltage,
            final double shooterFeederVoltage
    ) {
        return run(() -> intakeIO.toVoltage(
                rightRollerVoltage,
                leftRollerVoltage,
                shooterFeederVoltage
        ));
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
                        voltageMeasure -> intakeIO.toTorqueCurrent(
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

    @SuppressWarnings("unused")
    public Command runSysIDRoutineTorqueCurrent() {
        return Commands.sequence(
                torqueCurrentSysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward),
                Commands.waitSeconds(4),
                torqueCurrentSysIdRoutine.quasistatic(SysIdRoutine.Direction.kReverse),
                Commands.waitSeconds(4),
                torqueCurrentSysIdRoutine.dynamic(SysIdRoutine.Direction.kForward),
                Commands.waitSeconds(4),
                torqueCurrentSysIdRoutine.dynamic(SysIdRoutine.Direction.kReverse),
                Commands.waitSeconds(4),
                Commands.runOnce(SignalLogger::stop)
        );
    }
}
