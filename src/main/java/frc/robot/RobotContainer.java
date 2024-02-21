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
import frc.robot.subsystems.drive.Swerve;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.superstructure.arm.Arm;
import frc.robot.subsystems.superstructure.shooter.Shooter;

public class RobotContainer {
    public final PowerDistribution powerDistribution;

//    public final Swerve swerve;
    public final Arm arm;
    public final Shooter shooter;

    public final Superstructure superstructure;

    public final CommandXboxController driverController;
    public final CommandXboxController coDriverController;

    public RobotContainer() {
        this.powerDistribution = new PowerDistribution(
                RobotMap.PowerDistributionHub,
                PowerDistribution.ModuleType.kCTRE
        );
        this.powerDistribution.clearStickyFaults();

//        this.swerve = new Swerve(
//                Constants.RobotMode.REPLAY,
//                HardwareConstants.GYRO,
//                HardwareConstants.FRONT_LEFT_MODULE,
//                HardwareConstants.FRONT_RIGHT_MODULE,
//                HardwareConstants.BACK_LEFT_MODULE,
//                HardwareConstants.BACK_RIGHT_MODULE
//        );

        this.arm = new Arm(Constants.RobotMode.REPLAY, HardwareConstants.ARM);
        this.shooter = new Shooter(Constants.CURRENT_MODE, HardwareConstants.SHOOTER);

        this.superstructure = new Superstructure(arm, shooter);

        this.driverController = new CommandXboxController(RobotMap.MainController);
        this.coDriverController = new CommandXboxController(RobotMap.CoController);

        this.coDriverController.y().whileTrue(shooter.voltageSysIdQuasistaticTestCommand(SysIdRoutine.Direction.kForward));
        this.coDriverController.a().whileTrue(shooter.voltageSysIdQuasistaticTestCommand(SysIdRoutine.Direction.kReverse));
        this.coDriverController.b().whileTrue(shooter.voltageSysIdDynamicTestCommand(SysIdRoutine.Direction.kForward));
        this.coDriverController.x().whileTrue(shooter.voltageSysIdDynamicTestCommand(SysIdRoutine.Direction.kReverse));
        this.coDriverController.leftBumper().onTrue(Commands.runOnce(SignalLogger::stop));
    }

    public Command getAutonomousCommand() {
        return superstructure.cook();
    }
}
