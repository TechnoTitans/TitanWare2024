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
import frc.robot.constants.FieldConstants;
import frc.robot.constants.HardwareConstants;
import frc.robot.constants.RobotMap;
import frc.robot.subsystems.drive.Swerve;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.superstructure.ShotParameters;
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

    public final Autos autos;
    private final AutoChooser<String, AutoOption> autoChooser;

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

        configureAutos();
        configureButtonBindings();
    }

    public Command runEjectShooter() {
        return Commands.parallel(
                superstructure.toGoal(Superstructure.Goal.EJECT),
                intake
                        .runStopCommand()
                        .until(superstructure.atGoalTrigger)
                        .withTimeout(6)
                        .andThen(intake.runEjectInCommand()
                                .withTimeout(4))
        );
    }

    public Command runEjectIntake() {
        return Commands.parallel(
                superstructure.toGoal(Superstructure.Goal.BACK_FEED),
                intake
                        .runStopCommand()
                        .until(superstructure.atGoalTrigger)
                        .withTimeout(6)
                        .andThen(intake.runEjectOutCommand())
        );
    }

    public Command amp() {
        return Commands.sequence(
                arm.toGoal(Arm.Goal.AMP)
                        .until(arm.atPivotSetpoint),
                shooter.toGoal(Shooter.Goal.AMP)
                        .until(shooter.atVelocitySetpoint)
                        .withTimeout(4),
                intake.feedCommand()
        );
    }

    public Command stopAndShoot() {
        return Commands.deadline(
                Commands.parallel(
                        superstructure.toState(() -> ShotParameters.get(
                                swerve.getPose()
                                        .minus(FieldConstants.getSpeakerPose())
                                        .getTranslation()
                                        .getNorm())
                        ),
                        intake
                                .runStopCommand()
                                .until(superstructure.atGoalTrigger.and(swerve.atHeadingSetpoint))
                                .andThen(intake.feedCommand())
                ),
                swerve.teleopFacingAngleCommand(
                        () -> 0,
                        () -> 0,
                        () -> swerve.getPose()
                                .getTranslation()
                                .minus(FieldConstants.getSpeakerPose().getTranslation())
                                .getAngle()
                )
        );
    }

    public Command teleopDriveAimAndShoot() {
        //noinspection SuspiciousNameCombination
        return Commands.deadline(
                Commands.parallel(
                        superstructure.toState(() -> ShotParameters.get(
                                swerve.getPose()
                                        .minus(FieldConstants.getSpeakerPose())
                                        .getTranslation()
                                        .getNorm())
                        ).until(superstructure.atGoalTrigger),
                        intake
                                .runStopCommand()
                                .until(superstructure.atGoalTrigger.and(swerve.atHeadingSetpoint))
                                .andThen(intake.feedCommand())
                ),
                swerve.teleopFacingAngleCommand(
                        driverController::getLeftY,
                        driverController::getLeftX,
                        () -> swerve.getPose()
                                .getTranslation()
                                .minus(FieldConstants.getSpeakerPose().getTranslation())
                                .getAngle()
                )
        );
    }

    public void configureAutos() {
        autoChooser.addAutoOption(new AutoOption(
                "SourceSpeaker0",
                autos.sourceSpeaker0(),
                Constants.CompetitionType.COMPETITION
        ));
        autoChooser.addAutoOption(new AutoOption(
                "SourceSpeaker0Center1",
                autos.sourceSpeaker0Center1(),
                Constants.CompetitionType.COMPETITION
        ));
        autoChooser.addAutoOption(new AutoOption(
                "SourceSpeaker0Center1_2",
                autos.sourceSpeaker0Center1_2(),
                Constants.CompetitionType.COMPETITION
        ));
        autoChooser.addAutoOption(new AutoOption(
                "Walton",
                autos.walton(),
                Constants.CompetitionType.COMPETITION
        ));
        autoChooser.addAutoOption(new AutoOption(
                "Speaker0_1_2",
                autos.speaker_0_1_2(),
                Constants.CompetitionType.COMPETITION
        ));
        autoChooser.addAutoOption(new AutoOption(
                "AmpSpeaker2Center3",
                autos.ampSpeaker2Center3(),
                Constants.CompetitionType.COMPETITION
        ));
    }

    public void configureButtonBindings() {
        this.driverController.leftTrigger().whileTrue(intake.intakeCommand());
        this.driverController.rightTrigger().whileTrue(stopAndShoot());

        this.driverController.a().whileTrue(amp());

        this.driverController.y().onTrue(swerve.zeroRotationCommand());

        this.driverController.leftBumper().whileTrue(
                Commands.startEnd(
                        () -> Profiler.setSwerveSpeed(Profiler.SwerveSpeed.FAST),
                        () -> Profiler.setSwerveSpeed(Profiler.SwerveSpeed.NORMAL)
                )
        );

        this.driverController.rightBumper().whileTrue(
                Commands.startEnd(
                        () -> Profiler.setSwerveSpeed(Profiler.SwerveSpeed.SLOW),
                        () -> Profiler.setSwerveSpeed(Profiler.SwerveSpeed.NORMAL)
                )
        );

        this.coDriverController.y().whileTrue(runEjectShooter());
        this.coDriverController.a().whileTrue(runEjectIntake());
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected().autoCommand();
    }
}
