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
    public final Trigger noteJammed;

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
        this.noteJammed = new Trigger(() -> inputs.leftTorqueCurrentAmps >= 40 || inputs.rightTorqueCurrentAmps >= 40);

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
        Logger.recordOutput(LogKey + "/Setpoint/RightRollerVelocityRotsPerSec", setpoint.rightRollerVelocityRotsPerSec);
        Logger.recordOutput(LogKey + "/Setpoint/LeftRollerVelocityRotsPerSec", setpoint.leftRollerVelocityRotsPerSec);
        Logger.recordOutput(LogKey + "/Setpoint/ShooterFeederRotsPerSec", setpoint.shooterFeederRotsPerSec);
    }

    public Command intakeCommand() {
        return toVelocityCommand(9, 9, 9)
                .until(shooterBeamBreakBroken.or(noteJammed))
                .andThen(Commands.either(
                        outtakeCommand().until(noteJammed.negate()).withTimeout(5),
                        stopCommand(),
                        noteJammed
                ));
    }

    public Command feedCommand() {
        return toVelocityCommand(9, 9, 9)
                .until(shooterBeamBreakBroken.negate())
                .andThen(stopCommand());
    }

    public Command outtakeCommand() {
        return toVoltageCommand(-12, -12, -12);
    }

    public Command stopCommand() {
        return toVoltageCommand(0, 0, 0);
    }

    public Command toVelocityCommand(
            final double rightRollerVelocityRotsPerSec,
            final double leftRollerVelocityRotsPerSec,
            final double shooterFeederRotsPerSec
    ) {
        return startEnd(
                () -> {
                    setpoint.rightRollerVelocityRotsPerSec = rightRollerVelocityRotsPerSec;
                    setpoint.leftRollerVelocityRotsPerSec = leftRollerVelocityRotsPerSec;
                    setpoint.shooterFeederRotsPerSec = shooterFeederRotsPerSec;

                    intakeIO.toVelocity(
                            rightRollerVelocityRotsPerSec,
                            leftRollerVelocityRotsPerSec,
                            shooterFeederRotsPerSec
                    );
                },
                () -> {
                    setpoint.rightRollerVelocityRotsPerSec = 0;
                    setpoint.leftRollerVelocityRotsPerSec = 0;
                    setpoint.shooterFeederRotsPerSec = 0;

                    intakeIO.toVelocity(
                            rightRollerVelocityRotsPerSec,
                            leftRollerVelocityRotsPerSec,
                            shooterFeederRotsPerSec
                    );
                }
        );
    }

    public Command toVoltageCommand(
            final double rightRollerVoltage,
            final double leftRollerVoltage,
            final double shooterFeederVoltage
    ) {
        return startEnd(
                () -> intakeIO.toVoltage(
                        rightRollerVoltage,
                        leftRollerVoltage,
                        shooterFeederVoltage
                ),
                () -> intakeIO.toVoltage(
                        0,
                        0,
                        0
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
