package frc.robot;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.constants.Constants;
import frc.robot.constants.HardwareConstants;
import frc.robot.constants.RobotMap;
import frc.robot.subsystems.shooter.Shooter;

public class RobotContainer {
    public final PowerDistribution powerDistribution;

//    public final Swerve swerve;
    public final Shooter shooter;

//    public final CommandXboxController driverController;
    public final CommandXboxController coDriverController;

    public RobotContainer() {
        this.powerDistribution = new PowerDistribution(
                RobotMap.PowerDistributionHub,
                PowerDistribution.ModuleType.kCTRE
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

//        this.driverController = new CommandXboxController(RobotMap.MainController);
        this.coDriverController = new CommandXboxController(RobotMap.CoController);

        this.coDriverController.y().whileTrue(shooter.sysIdQuasistaticTestCommand(SysIdRoutine.Direction.kForward));
        this.coDriverController.a().whileTrue(shooter.sysIdQuasistaticTestCommand(SysIdRoutine.Direction.kReverse));
        this.coDriverController.b().whileTrue(shooter.sysIdDynamicTestCommand(SysIdRoutine.Direction.kForward));
        this.coDriverController.x().whileTrue(shooter.sysIdDynamicTestCommand(SysIdRoutine.Direction.kReverse));

        this.coDriverController.leftBumper().onTrue(Commands.run(SignalLogger::stop));
    }

    public Command getAutonomousCommand() {
        return Commands.waitUntil(() -> !RobotState.isAutonomous());
    }
}
