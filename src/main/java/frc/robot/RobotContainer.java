package frc.robot;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.constants.Constants;
import frc.robot.constants.HardwareConstants;
import frc.robot.constants.RobotMap;
import frc.robot.subsystems.drive.Swerve;
import frc.robot.subsystems.intake.Intake;

public class RobotContainer {
    public final PowerDistribution powerDistribution;

    public final Swerve swerve;
    public final Intake intake;

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

        this.driverController = new CommandXboxController(RobotMap.MainController);
        this.coDriverController = new CommandXboxController(RobotMap.CoController);

        this.coDriverController.y().whileTrue(intake.torqueCurrentSysIdQuasistaticTestCommand(SysIdRoutine.Direction.kForward));
        this.coDriverController.a().whileTrue(intake.torqueCurrentSysIdQuasistaticTestCommand(SysIdRoutine.Direction.kReverse));
        this.coDriverController.b().whileTrue(intake.torqueCurrentSysIdDynamicTestCommand(SysIdRoutine.Direction.kForward));
        this.coDriverController.x().whileTrue(intake.torqueCurrentSysIdDynamicTestCommand(SysIdRoutine.Direction.kReverse));

        this.coDriverController.leftBumper().onTrue(Commands.run(SignalLogger::stop));
    }

    public Command getAutonomousCommand() {
//        return Commands.waitUntil(() -> !RobotState.isAutonomous());
        return intake.toVelocity(2, 2, 2);
    }
}
