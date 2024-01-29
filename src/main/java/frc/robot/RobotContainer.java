package frc.robot;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.constants.Constants;
import frc.robot.constants.HardwareConstants;
import frc.robot.constants.RobotMap;
import frc.robot.subsystems.drive.Swerve;
import frc.robot.subsystems.shooter.Shooter;

public class RobotContainer {
    public final PowerDistribution powerDistribution;

//    public final Swerve swerve;
    public final Shooter shooter;

    public final CommandXboxController driverController;
    public final CommandXboxController coDriverController;

    public RobotContainer() {
        this.powerDistribution = new PowerDistribution(
                RobotMap.PowerDistributionHub,
                PowerDistribution.ModuleType.kRev
        );
        this.powerDistribution.clearStickyFaults();

//        this.swerve = new Swerve(
//                Constants.CURRENT_MODE,
//                HardwareConstants.FRONT_LEFT_MODULE,
//                HardwareConstants.FRONT_RIGHT_MODULE,
//                HardwareConstants.BACK_LEFT_MODULE,
//                HardwareConstants.BACK_RIGHT_MODULE
//        );

        this.shooter = new Shooter(
                Constants.CURRENT_MODE,
                HardwareConstants.SHOOTER
        );

        this.driverController = new CommandXboxController(RobotMap.MainController);
        this.coDriverController = new CommandXboxController(RobotMap.CoController);
    }

    public Command getAutonomousCommand() {
        return Commands.waitUntil(() -> !RobotState.isAutonomous());
    }
}
