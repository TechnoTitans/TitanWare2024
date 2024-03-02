package frc.robot;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.constants.Constants;
import frc.robot.constants.HardwareConstants;
import frc.robot.constants.RobotMap;
import frc.robot.subsystems.drive.Swerve;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.superstructure.arm.Arm;
import frc.robot.subsystems.superstructure.shooter.Shooter;

public class RobotContainer {
    public final PowerDistribution powerDistribution;

    public final Swerve swerve;
    public final Arm arm;
    public final Shooter shooter;

    public final Superstructure superstructure;

    private final TalonFX intakeLeft;
    private final TalonFX intakeRight;

    public final CommandXboxController driverController;
    public final CommandXboxController coDriverController;

    public RobotContainer() {
        this.powerDistribution = new PowerDistribution(
                RobotMap.PowerDistributionHub,
                PowerDistribution.ModuleType.kRev
        );
        this.powerDistribution.clearStickyFaults();

        this.swerve = new Swerve(
                Constants.CURRENT_MODE,
                HardwareConstants.GYRO,
                HardwareConstants.FRONT_LEFT_MODULE,
                HardwareConstants.FRONT_RIGHT_MODULE,
                HardwareConstants.BACK_LEFT_MODULE,
                HardwareConstants.BACK_RIGHT_MODULE
        );

        this.arm = new Arm(Constants.RobotMode.REPLAY, HardwareConstants.ARM);
        this.shooter = new Shooter(Constants.CURRENT_MODE, HardwareConstants.SHOOTER);

        this.superstructure = new Superstructure(arm, shooter);

        this.intakeLeft = new TalonFX(20);
        this.intakeRight = new TalonFX(19);

        this.intakeLeft.getConfigurator().apply(new TalonFXConfiguration()
                .withCurrentLimits(new CurrentLimitsConfigs()
                                .withStatorCurrentLimit(60)
                                .withStatorCurrentLimitEnable(true)
                )
                .withMotorOutput(new MotorOutputConfigs()
                        .withNeutralMode(NeutralModeValue.Coast)
                )
        );
        this.intakeRight.getConfigurator().apply(new TalonFXConfiguration()
                .withCurrentLimits(new CurrentLimitsConfigs()
                        .withStatorCurrentLimit(60)
                        .withStatorCurrentLimitEnable(true)
                )
                .withMotorOutput(new MotorOutputConfigs()
                        .withNeutralMode(NeutralModeValue.Coast)
                )
        );

        this.driverController = new CommandXboxController(RobotMap.MainController);
        this.coDriverController = new CommandXboxController(RobotMap.CoController);

        final VoltageOut voltageOut = new VoltageOut(0);
        this.driverController.y().onTrue(
                Commands.sequence(
                        superstructure.toGoal(Superstructure.Goal.IDLE),
//                        Commands.waitUntil(superstructure.atGoalTrigger),
                        Commands.waitSeconds(6),
                        superstructure.toGoal(Superstructure.Goal.SUBWOOFER),
//                        Commands.waitUntil(superstructure.atGoalTrigger),
                        Commands.waitUntil(superstructure.getShooter().atVelocityTrigger),
                        Commands.runOnce(() -> {
                            intakeLeft.setControl(voltageOut.withOutput(10));
                            intakeRight.setControl(voltageOut.withOutput(10));
                        }),
                        Commands.waitSeconds(2),
                        Commands.runOnce(() -> {
                            intakeLeft.setControl(voltageOut.withOutput(0));
                            intakeRight.setControl(voltageOut.withOutput(0));
                        }),
                        Commands.waitSeconds(2),
                        superstructure.toGoal(Superstructure.Goal.IDLE)
                )
        );
    }

    public Command getAutonomousCommand() {
        return Commands.repeatingSequence(
                superstructure.toGoal(Superstructure.Goal.IDLE),
                Commands.waitUntil(superstructure.atGoalTrigger),
                Commands.waitSeconds(4),
                superstructure.toGoal(Superstructure.Goal.SUBWOOFER),
                Commands.waitUntil(superstructure.atGoalTrigger),
                Commands.waitSeconds(4)
        );
    }
}
