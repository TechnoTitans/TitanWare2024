package frc.robot;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.constants.Constants;
import frc.robot.constants.HardwareConstants;
import frc.robot.constants.RobotMap;
import frc.robot.subsystems.drive.Swerve;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.superstructure.arm.Arm;
import frc.robot.subsystems.superstructure.shooter.Shooter;

public class RobotContainer {
    public final PowerDistribution powerDistribution;

    public final Swerve swerve;
    public final Intake intake;

    public final Arm arm;
    public final Shooter shooter;

    public final Superstructure superstructure;

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

        this.intake = new Intake(
                Constants.CURRENT_MODE,
                HardwareConstants.INTAKE,
                swerve::getFieldRelativeSpeeds
        );

        this.arm = new Arm(Constants.RobotMode.REPLAY, HardwareConstants.ARM);
        this.shooter = new Shooter(Constants.CURRENT_MODE, HardwareConstants.SHOOTER);

        this.superstructure = new Superstructure(arm, shooter);

        this.driverController = new CommandXboxController(RobotMap.MainController);
        this.coDriverController = new CommandXboxController(RobotMap.CoController);

        this.driverController.y().onTrue(
                Commands.sequence(
                        superstructure.toGoal(Superstructure.Goal.IDLE),
//                        Commands.waitUntil(superstructure.atGoalTrigger),
                        Commands.waitSeconds(6),
                        superstructure.toGoal(Superstructure.Goal.SUBWOOFER),
//                        Commands.waitUntil(superstructure.atGoalTrigger),
                        Commands.waitUntil(superstructure.getShooter().atVelocityTrigger),
                        Commands.waitSeconds(4),
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
