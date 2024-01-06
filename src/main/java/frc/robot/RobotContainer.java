package frc.robot;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.constants.Constants;
import frc.robot.constants.HardwareConstants;
import frc.robot.constants.RobotMap;
import frc.robot.subsystems.drive.Swerve;

public class RobotContainer {
    public final PowerDistribution powerDistribution;

    public final Swerve swerve;

    public RobotContainer() {
        powerDistribution = new PowerDistribution(RobotMap.PowerDistributionHub, PowerDistribution.ModuleType.kRev);
        powerDistribution.clearStickyFaults();

        swerve = new Swerve(
                Constants.CURRENT_MODE,
                HardwareConstants.FRONT_LEFT_MODULE,
                HardwareConstants.FRONT_RIGHT_MODULE,
                HardwareConstants.BACK_LEFT_MODULE,
                HardwareConstants.BACK_RIGHT_MODULE
        );
    }

    public Command getAutonomousCommand() {
        return Commands.waitUntil(() -> !RobotState.isAutonomous());
    }
}
