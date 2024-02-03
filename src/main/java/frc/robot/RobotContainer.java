package frc.robot;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;
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

public class RobotContainer {
    public final PowerDistribution powerDistribution;

    public final Swerve swerve;

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
                HardwareConstants.FRONT_LEFT_MODULE,
                HardwareConstants.FRONT_RIGHT_MODULE,
                HardwareConstants.BACK_LEFT_MODULE,
                HardwareConstants.BACK_RIGHT_MODULE
        );

        this.driverController = new CommandXboxController(RobotMap.MainController);
        this.coDriverController = new CommandXboxController(RobotMap.CoController);

        this.coDriverController.y().whileTrue(swerve.linearVoltageSysIdQuasistaticCommand(SysIdRoutine.Direction.kForward));
        this.coDriverController.a().whileTrue(swerve.linearVoltageSysIdQuasistaticCommand(SysIdRoutine.Direction.kReverse));
        this.coDriverController.b().whileTrue(swerve.linearVoltageSysIdDynamicCommand(SysIdRoutine.Direction.kForward));
        this.coDriverController.x().whileTrue(swerve.linearVoltageSysIdDynamicCommand(SysIdRoutine.Direction.kReverse));

        this.coDriverController.leftBumper().onTrue(Commands.run(SignalLogger::stop));
    }

    public Command getAutonomousCommand() {
//        return Commands.waitUntil(() -> !RobotState.isAutonomous());
        final ChoreoTrajectory trajectory = Choreo.getTrajectory("LineTest");
        return Commands.sequence(
                Commands.runOnce(() -> swerve.resetPosition(trajectory.getInitialPose())),
                swerve.followChoreoPathCommand(trajectory)
        );
    }
}
