package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.constants.Constants;
import frc.robot.constants.HardwareConstants;
import frc.robot.utils.logging.LogUtils;
import frc.robot.utils.logging.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.Volts;

public class Shooter extends SubsystemBase {
    protected static final String logKey = "Shooter";

    private final ShooterIO shooterIO;
    private final ShooterIOInputsAutoLogged inputs;

    private final SysIdRoutine shooterSysId;

    private final LoggedTunableNumber topShooterVelocityTuner = new LoggedTunableNumber(
            logKey + "/TopShooterVelocity", 0
    );
    private final LoggedTunableNumber bottomShooterVelocityTuner = new LoggedTunableNumber(
            logKey + "/BottomShooterVelocity", 0
    );

    public Shooter(
            final Constants.RobotMode mode,
            final HardwareConstants.ShooterConstants shooterConstants
    ) {
        this.shooterIO = switch (mode) {
            case REAL -> new ShooterIOReal(shooterConstants);
            case SIM -> new ShooterIOSim(shooterConstants);
            case REPLAY -> new ShooterIO() {};
        };

        this.inputs = new ShooterIOInputsAutoLogged();
        this.shooterSysId = new SysIdRoutine(
                new SysIdRoutine.Config(
                        null,
                        null,
                        null,
                        state -> SignalLogger.writeString("state", state.toString())
                ),
                new SysIdRoutine.Mechanism(
                        voltageMeasure -> shooterIO.setCharacterizationVolts(voltageMeasure.in(Volts), voltageMeasure.in(Volts)),
                        null,
                        this
                )
        );

        this.shooterIO.config();
        this.shooterIO.initialize();
    }

    public void periodic() {
        shooterIO.updateInputs(inputs);
        shooterIO.periodic();

        final double shooterPeriodicUpdateStart = Logger.getRealTimestamp();

        Logger.processInputs(logKey, inputs);

        Logger.recordOutput(
                logKey + "/PeriodicIOPeriodMs",
                LogUtils.microsecondsToMilliseconds(Logger.getRealTimestamp() - shooterPeriodicUpdateStart)
        );
    }

    public Command tunableNumbersInputCommand() {
        return run(() -> shooterIO.setInputs(
                topShooterVelocityTuner.get(),
                bottomShooterVelocityTuner.get()
        ));
    }

    public Command sysIdQuasistaticTestCommand(final SysIdRoutine.Direction direction) {
        return shooterSysId.quasistatic(direction);
    }

    public Command sysIdDynamicTestCommand(final SysIdRoutine.Direction direction) {
        return shooterSysId.dynamic(direction);
    }
}
