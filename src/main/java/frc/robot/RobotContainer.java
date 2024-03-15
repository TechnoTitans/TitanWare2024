package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.event.EventLoop;
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
import frc.robot.subsystems.vision.PhotonVision;
import frc.robot.utils.teleop.Profiler;

public class RobotContainer {
    public final PowerDistribution powerDistribution;

    public final Swerve swerve;
    public final Intake intake;

    public final Arm arm;
    public final Shooter shooter;

    public final Superstructure superstructure;

    public final ShootCommands shootCommands;
    public final Autos autos;
    public final AutoChooser<String, AutoOption> autoChooser;

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
        this.shootCommands = new ShootCommands(swerve, intake, superstructure);
        this.autos = new Autos(swerve, intake, superstructure, shootCommands);

        this.driverController = new CommandXboxController(RobotMap.MainController);
        this.coDriverController = new CommandXboxController(RobotMap.CoController);

        this.autoChooser = new AutoChooser<>(
                new AutoOption(
                        "DoNothing",
                        autos.doNothing(),
                        Constants.CompetitionType.COMPETITION
                )
        );
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

    public void configureAutos() {
        autoChooser.addAutoOption(new AutoOption(
                "SourceSpeaker0",
                autos.sourceSpeaker0(),
                Constants.CompetitionType.COMPETITION
        ));
        autoChooser.addAutoOption(new AutoOption(
                "AmpSpeaker0",
                autos.ampSpeaker0(),
                Constants.CompetitionType.COMPETITION
        ));
        autoChooser.addAutoOption(new AutoOption(
                "SourceSpeaker0Center1",
                autos.sourceSpeaker0Center1(),
                Constants.CompetitionType.TESTING
        ));
        autoChooser.addAutoOption(new AutoOption(
                "SourceSpeaker0Center1_2",
                autos.sourceSpeaker0Center1_2(),
                Constants.CompetitionType.TESTING
        ));
        autoChooser.addAutoOption(new AutoOption(
                "Walton",
                autos.walton(),
                Constants.CompetitionType.TESTING
        ));
        autoChooser.addAutoOption(new AutoOption(
                "Speaker0_1_2",
                autos.speaker0_1_2(),
                Constants.CompetitionType.COMPETITION
        ));
        autoChooser.addAutoOption(new AutoOption(
                "AmpSpeaker2Center3",
                autos.ampSpeaker2Center3(),
                Constants.CompetitionType.TESTING
        ));
        autoChooser.addAutoOption(new AutoOption(
                "ShootAndMobility",
                autos.shootAndMobility(),
                Constants.CompetitionType.COMPETITION
        ));
    }

    public void configureButtonBindings(final EventLoop teleopEventLoop) {
        final XboxController driverHID = driverController.getHID();
        this.driverController.leftTrigger(0.5, teleopEventLoop).whileTrue(
                intake.intakeCommand(
                        Commands.startEnd(
                                () -> driverHID.setRumble(GenericHID.RumbleType.kBothRumble, 0.5),
                                () -> driverHID.setRumble(GenericHID.RumbleType.kBothRumble, 0)
                        ).withTimeout(0.5)
                )
        );
        this.driverController.rightTrigger(0.5, teleopEventLoop)
                .whileTrue(shootCommands.stopAimAndShoot())
                .onFalse(superstructure.toGoal(Superstructure.Goal.IDLE));

//        this.driverController.x().whileTrue(driveAndAmp());

//        this.coDriverController.b().whileTrue(swerve.faceAngle(() ->
//                swerve.getPose()
//                        .getTranslation()
//                        .minus(FieldConstants.getSpeakerPose().getTranslation())
//                        .getAngle()
//        ));

        this.driverController.a(teleopEventLoop).whileTrue(shootCommands.amp());
        this.driverController.y(teleopEventLoop).onTrue(swerve.zeroRotationCommand());

        this.driverController.leftBumper(teleopEventLoop).whileTrue(
                Commands.startEnd(
                        () -> Profiler.setSwerveSpeed(Profiler.SwerveSpeed.FAST),
                        () -> Profiler.setSwerveSpeed(Profiler.SwerveSpeed.NORMAL)
                )
        );

        this.driverController.rightBumper(teleopEventLoop).whileTrue(
                Commands.startEnd(
                        () -> Profiler.setSwerveSpeed(Profiler.SwerveSpeed.SLOW),
                        () -> Profiler.setSwerveSpeed(Profiler.SwerveSpeed.NORMAL)
                )
        );

        this.coDriverController.y(teleopEventLoop).whileTrue(runEjectShooter());
        this.coDriverController.a(teleopEventLoop).whileTrue(runEjectIntake());
    }

    public EventLoop getAutonomousEventLoop() {
        return autoChooser.getSelected().autoEventLoop();
//        return autos.shootAndMobility();
    }
}
