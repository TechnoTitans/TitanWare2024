package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.auto.AutoChooser;
import frc.robot.auto.AutoOption;
import frc.robot.auto.Autos;
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

    public final Autos autos;
    private final AutoChooser<String, AutoOption> autoChooser;

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

        this.arm = new Arm(Constants.CURRENT_MODE, HardwareConstants.ARM);
        this.shooter = new Shooter(Constants.CURRENT_MODE, HardwareConstants.SHOOTER);

        this.superstructure = new Superstructure(arm, shooter);

        this.autos = new Autos(swerve);

        this.driverController = new CommandXboxController(RobotMap.MainController);
        this.coDriverController = new CommandXboxController(RobotMap.CoController);

        this.autoChooser = new AutoChooser<>(
                new AutoOption(
                        "DoNothing",
                        Commands.waitUntil(() -> !DriverStation.isAutonomousEnabled()),
                        Constants.CompetitionType.COMPETITION
                )
        );
        autoChooser.addAutoOption(new AutoOption(
                "PreloadAndBack",
                autos.sourceSpeaker0(),
                Constants.CompetitionType.COMPETITION
        ));
        autoChooser.addAutoOption(new AutoOption(
                "PreloadBackCenter1",
                autos.sourceSpeaker0Center1(),
                Constants.CompetitionType.COMPETITION
        ));
        autoChooser.addAutoOption(new AutoOption(
                "PreloadBackCenter2",
                autos.sourceSpeaker0Center1_2(),
                Constants.CompetitionType.COMPETITION
        ));
        autoChooser.addAutoOption(new AutoOption(
                "Walton",
                autos.walton(),
                Constants.CompetitionType.COMPETITION
        ));
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected().autoCommand();
    }
}
