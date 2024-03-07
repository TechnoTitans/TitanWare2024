package frc.robot;

import com.ctre.phoenix6.SignalLogger;
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
import frc.robot.subsystems.vision.PhotonVision;
import frc.robot.utils.teleop.Profiler;

public class RobotContainer {
    public final PowerDistribution powerDistribution;

    public final Swerve swerve;
    public final Intake intake;

    public final Arm arm;
    public final Shooter shooter;

    public final Superstructure superstructure;

    public final PhotonVision photonVision;

    public final CommandXboxController driverController;
    public final CommandXboxController coDriverController;

    public RobotContainer() {
        this.powerDistribution = new PowerDistribution(
                RobotMap.PowerDistributionHub,
                PowerDistribution.ModuleType.kRev
        );
        this.powerDistribution.clearStickyFaults();
        this.powerDistribution.setSwitchableChannel(true);

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
                HardwareConstants.INTAKE
        );

        this.arm = new Arm(Constants.CURRENT_MODE, HardwareConstants.ARM);
        this.shooter = new Shooter(Constants.CURRENT_MODE, HardwareConstants.SHOOTER);

        this.superstructure = new Superstructure(arm, shooter);

        this.photonVision = new PhotonVision(Constants.CURRENT_MODE, swerve, swerve.getPoseEstimator());

        this.driverController = new CommandXboxController(RobotMap.MainController);
        this.coDriverController = new CommandXboxController(RobotMap.CoController);

        driverController.y().onTrue(swerve.zeroCommand());

//        intake.toVoltageCommand(6, 6, 6);
//        driverController.b().onTrue(
//                intake.toVoltageCommand(8, 8, 8)
//        );
//        shooter.setDefaultCommand(shooter.toVoltageCommand(7, -6, -6));

        driverController.leftBumper().whileTrue(
                Commands.sequence(
                        Commands.parallel(
                                arm.toGoal(Arm.Goal.SUBWOOFER),
                                shooter.toVoltageCommand(10, 10, 10)
                        ),
                        Commands.waitSeconds(2),
                        intake.toVoltageCommand(9, 9, 9)
                )
        ).onFalse(
                Commands.parallel(
                        arm.toGoal(Arm.Goal.STOW),
                        shooter.toGoal(Shooter.Goal.IDLE),
                        intake.toVoltageCommand(9, 9, 0)
                )
        );


        driverController.leftBumper().whileTrue(
                Commands.startEnd(
                        () -> Profiler.setSwerveSpeed(Profiler.SwerveSpeed.FAST),
                        () -> Profiler.setSwerveSpeed(Profiler.SwerveSpeed.NORMAL)
                )
        );

        driverController.rightBumper().whileTrue(
                Commands.startEnd(
                        () -> Profiler.setSwerveSpeed(Profiler.SwerveSpeed.SLOW),
                        () -> Profiler.setSwerveSpeed(Profiler.SwerveSpeed.NORMAL)
                )
        );

    }

    public Command getAutonomousCommand() {
        return Commands.sequence(
                arm.homePivotCurrentCommand(),
                Commands.repeatingSequence(
                        arm.toGoal(Arm.Goal.STOW),
                        Commands.waitSeconds(2),
                        arm.toGoal(Arm.Goal.SUBWOOFER),
                        Commands.waitSeconds(2),
                        arm.toGoal(Arm.Goal.AMP),
                        Commands.waitSeconds(4)
                )
        );
    }
}
