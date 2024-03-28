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
import frc.robot.state.NoteState;
import frc.robot.subsystems.climb.Climb;
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
    public final Climb climb;

    public final Superstructure superstructure;

    public final NoteState noteState;

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
        this.climb = new Climb(Constants.CURRENT_MODE, HardwareConstants.CLIMB);
        this.superstructure = new Superstructure(arm, shooter);

        this.noteState = new NoteState(intake, superstructure);

        this.photonVision = new PhotonVision(Constants.CURRENT_MODE, swerve, swerve.getPoseEstimator());
        this.shootCommands = new ShootCommands(swerve, intake, superstructure);
        this.autos = new Autos(swerve, intake, superstructure, noteState, shootCommands);

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
        return Commands.parallel(
                intake.runEjectInCommand(),
                superstructure.toGoal(Superstructure.Goal.EJECT)
        );
    }

    public Command runEjectIntake() {
        return Commands.deadline(
                intake.runEjectOutCommand(),
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
                "AmpSpeaker2",
                autos.ampSpeaker2(),
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
                "Speaker2_1_0",
                autos.speaker2_1_0(),
                Constants.CompetitionType.COMPETITION
        ));
        autoChooser.addAutoOption(new AutoOption(
                "AmpSpeaker2Center2_3",
                autos.ampSpeaker2Center2_3(),
                Constants.CompetitionType.COMPETITION
        ));
        autoChooser.addAutoOption(new AutoOption(
                "SourceMobility",
                autos.sourceMobility(),
                Constants.CompetitionType.COMPETITION
        ));
        autoChooser.addAutoOption(new AutoOption(
                "SourceCenter1_0",
                autos.sourceCenter1_0(),
                Constants.CompetitionType.COMPETITION
        ));
        autoChooser.addAutoOption(new AutoOption(
                "AmpCenter3_2",
                autos.ampCenter3_2(),
                Constants.CompetitionType.COMPETITION
        ));
        autoChooser.addAutoOption(new AutoOption(
                "AmpCenter4_3",
                autos.ampCenter4_3(),
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
//                .whileTrue(shootCommands.stopAimAndShoot())
                .whileTrue(shootCommands.deferredStopAimAndShoot())
                .onFalse(superstructure.toGoal(Superstructure.Goal.IDLE));

        this.driverController.a(teleopEventLoop).whileTrue(shootCommands.amp());
        this.driverController.y(teleopEventLoop).onTrue(swerve.zeroRotationCommand());
        this.driverController.b(teleopEventLoop).whileTrue(climb.climbCommand());

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
        this.coDriverController.b().whileTrue(shootCommands.lineupAndAmp());
        this.coDriverController.rightTrigger(0.5, teleopEventLoop)
                .whileTrue(shootCommands.shootSubwoofer());
    }

    public EventLoop getAutonomousEventLoop() {
        return autoChooser.getSelected().autoEventLoop();
//        return autos.shootAndMobility();
    }
}
