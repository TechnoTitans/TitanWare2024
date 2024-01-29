package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.Slot0Configs;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.constants.HardwareConstants;
import frc.robot.utils.logging.LogUtils;
import frc.robot.utils.logging.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
    protected static final String logKey = "Shooter";

    private final ShooterIO shooterIO;
    private final ShooterIOInputsAutoLogged inputs;

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

        this.shooterIO.config();
        this.shooterIO.initialize();
    }

    public void periodic() {
        shooterIO.updateInputs(inputs);
        shooterIO.periodic();

        final double modulePeriodicUpdateStart = Logger.getRealTimestamp();

        Logger.processInputs(logKey, inputs);

        Logger.recordOutput(
                logKey + "/PeriodicIOPeriodMs",
                LogUtils.microsecondsToMilliseconds(Logger.getRealTimestamp() - modulePeriodicUpdateStart)
        );
    }

    public Command setInputsFromTuner() {
        return Commands.run(() -> shooterIO.setInputs(
                topShooterVelocityTuner.get(), bottomShooterVelocityTuner.get()
        ), this);
    }
}
