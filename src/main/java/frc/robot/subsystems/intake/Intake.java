package frc.robot.subsystems.intake;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.units.Current;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Time;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.constants.Constants;
import frc.robot.constants.HardwareConstants;
import frc.robot.utils.logging.LogUtils;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.*;

public class Intake extends SubsystemBase {
    protected static final String logKey = "Intake";

    final IntakeIO intakeIO;
    final IntakeIOInputsAutoLogged inputs;

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
        intakeIO.periodic();

        Logger.recordOutput(
                logKey + "/PeriodicIOPeriodMs",
                LogUtils.microsecondsToMilliseconds(Logger.getRealTimestamp() - intakeIOPeriodicStart)
        );

        intakeIO.updateInputs(inputs);
        Logger.processInputs(logKey, inputs);
    }

    //TODO: this will eventually take an enum probably
    public void setState() {
        intakeIO.toVelocity(
                0,
                0,
                0
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
                        voltageMeasure -> intakeIO.setCharacterizationTorqueCurrent(
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
    public Command torqueCurrentSysIdQuasistaticTestCommand(final SysIdRoutine.Direction direction) {
        return torqueCurrentSysIdRoutine.quasistatic(direction);
    }

    @SuppressWarnings("unused")
    public Command torqueCurrentSysIdDynamicTestCommand(final SysIdRoutine.Direction direction) {
        return torqueCurrentSysIdRoutine.dynamic(direction);
    }
}
