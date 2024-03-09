package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.XboxController;
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
        this.autos = new Autos(swerve, superstructure, intake);

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
        return Commands.deadline(
                intake
                        .runStopCommand()
                        .until(superstructure.atSetpoint)
                        .withTimeout(6)
                        .andThen(intake.runEjectInCommand()
                                .withTimeout(4)),
                superstructure.toGoal(Superstructure.Goal.EJECT)
        );
    }

    public Command runEjectIntake() {
        return Commands.deadline(
                intake
                        .runStopCommand()
                        .until(superstructure.atSetpoint)
                        .withTimeout(6)
                        .andThen(intake.runEjectOutCommand()),
                superstructure.toGoal(Superstructure.Goal.BACK_FEED)
        );
    }

    public Command amp() {
        return Commands.deadline(
                Commands.sequence(
                        Commands.waitUntil(swerve.atHolonomicDrivePose)
                                .withTimeout(6),
                        intake.feedHalfCommand(),
                        Commands.deadline(
                                Commands.waitUntil(superstructure.atSetpoint)
                                        .andThen(Commands.waitSeconds(0.5))
                                        .andThen(intake.feedCommand())
                                        .andThen(Commands.waitSeconds(0.5)),
                                superstructure.toGoal(Superstructure.Goal.AMP)
                        )
                ),
                swerve.driveToPose(FieldConstants::getAmpScoringPose)
        );
    }

    public Command shootSubwoofer() {
        return Commands.deadline(
                intake
                        .runStopCommand()
                        .until(superstructure.atSetpoint)
                        .andThen(intake.feedCommand()),
                superstructure.toGoal(Superstructure.Goal.SUBWOOFER)
        );
    }

    public Command stopAndShoot() {
        return Commands.deadline(
                Commands.deadline(
                        intake
                                .runStopCommand()
                                .until(superstructure.atSetpoint.and(swerve.atHeadingSetpoint))
                                .withTimeout(4)
                                .andThen(intake.feedCommand()),
                        superstructure.runState(() -> ShotParameters.get(
                                swerve.getPose()
                                        .minus(FieldConstants.getSpeakerPose())
                                        .getTranslation()
                                        .getNorm()
                        ))
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
                Commands.deadline(
                        intake
                                .runStopCommand()
                                .until(superstructure.atSetpoint.and(swerve.atHeadingSetpoint))
                                .andThen(intake.feedCommand()),
                        superstructure.runState(() -> ShotParameters.get(
                                swerve.getPose()
                                        .minus(FieldConstants.getSpeakerPose())
                                        .getTranslation()
                                        .getNorm())
                        )
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
                autos.speaker0_1_2(),
                Constants.CompetitionType.COMPETITION
        ));
        autoChooser.addAutoOption(new AutoOption(
                "AmpSpeaker2Center3",
                autos.ampSpeaker2Center3(),
                Constants.CompetitionType.COMPETITION
        ));
        autoChooser.addAutoOption(new AutoOption(
                "ShootAndMobility",
                autos.shootAndMobility(),
                Constants.CompetitionType.COMPETITION
        ));
    }

    public void configureButtonBindings() {
        this.driverController.leftTrigger().whileTrue(amp());
        this.driverController.rightTrigger().whileTrue(shootSubwoofer());

        final XboxController driverHID = driverController.getHID();
        this.driverController.a().whileTrue(
                intake.intakeCommand(
                        Commands.startEnd(
                                () -> driverHID.setRumble(GenericHID.RumbleType.kBothRumble, 0.5),
                                () -> driverHID.setRumble(GenericHID.RumbleType.kBothRumble, 0)
                        ).withTimeout(0.5)
                )
        );
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
//        this.coDriverController.leftBumper().onTrue(Commands.runOnce(SignalLogger::stop));

//        this.coDriverController.y().whileTrue(swerve.linearTorqueCurrentSysIdQuasistaticCommand(SysIdRoutine.Direction.kForward));
//        this.coDriverController.a().whileTrue(swerve.linearTorqueCurrentSysIdQuasistaticCommand(SysIdRoutine.Direction.kReverse));
//        this.coDriverController.b().whileTrue(swerve.linearTorqueCurrentSysIdDynamicCommand(SysIdRoutine.Direction.kForward));
//        this.coDriverController.x().whileTrue(swerve.linearTorqueCurrentSysIdDynamicCommand(SysIdRoutine.Direction.kReverse));
    }

    public Command getAutonomousCommand() {
//        return autoChooser.getSelected().autoCommand();
        return Commands.sequence(
                arm.homePivotCommand(),
                autoChooser.getSelected().autoCommand()
        );
//        return autos.shootAndMobility();
    }
}
