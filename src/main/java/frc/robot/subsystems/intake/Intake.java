package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.constants.HardwareConstants;
import frc.robot.constants.SimConstants;
import frc.robot.utils.logging.LogUtils;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
    protected static final String logKey = "Intake";

    final IntakeIO intakeIO;
    final IntakeIOInputsAutoLogged inputs;

    public Intake(
            final Constants.RobotMode robotMode,
            final HardwareConstants.IntakeConstants intakeConstants
    ) {
        this.intakeIO = switch (robotMode) {
            case REAL -> new IntakeIOTalonFX(intakeConstants);
            case SIM -> new IntakeIOTalonFXSim(intakeConstants);
            case REPLAY -> new IntakeIO() {};
        };

        this.inputs = new IntakeIOInputsAutoLogged();

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
}
