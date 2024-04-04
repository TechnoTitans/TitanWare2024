package frc.robot;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.auto.AutoChooser;
import frc.robot.auto.AutoOption;
import frc.robot.auto.Autos;
import frc.robot.constants.Constants;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.HardwareConstants;
import frc.robot.constants.RobotMap;
import frc.robot.state.NoteState;
import frc.robot.subsystems.drive.Swerve;
import frc.robot.subsystems.drive.constants.SwerveConstants;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.superstructure.ShootOnTheMove;
import frc.robot.subsystems.superstructure.ShotParameters;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.superstructure.arm.Arm;
import frc.robot.subsystems.superstructure.shooter.Shooter;
import frc.robot.subsystems.vision.PhotonVision;
import frc.robot.utils.closeables.ToClose;
import frc.robot.utils.subsystems.VirtualSubsystem;
import frc.robot.utils.teleop.ControllerUtils;
import frc.robot.utils.teleop.Profiler;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.util.Optional;
import java.util.function.BooleanSupplier;

@SuppressWarnings("RedundantMethodOverride")
public class Robot extends LoggedRobot {
    private static final String AKitLogPath = "/U/logs";
    private static final String HootLogPath = "/U/logs";

    public static final BooleanSupplier IsRedAlliance = () -> {
        final Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
        return alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;
    };

    public final PowerDistribution powerDistribution = new PowerDistribution(
            RobotMap.PowerDistributionHub,
            PowerDistribution.ModuleType.kRev
    );

    public final Swerve swerve = new Swerve(
            Constants.CURRENT_MODE,
            HardwareConstants.GYRO,
            SwerveConstants.FrontLeftModule,
            SwerveConstants.FrontRightModule,
            SwerveConstants.BackLeftModule,
            SwerveConstants.BackRightModule
    );
    public final Intake intake = new Intake(Constants.CURRENT_MODE, HardwareConstants.INTAKE);
    public final Arm arm = new Arm(Constants.CURRENT_MODE, HardwareConstants.ARM);
    public final Shooter shooter = new Shooter(Constants.CURRENT_MODE, HardwareConstants.SHOOTER);
    public final Superstructure superstructure = new Superstructure(arm, shooter);

    @SuppressWarnings("unused")
    public final PhotonVision photonVision = new PhotonVision(Constants.CURRENT_MODE, swerve, swerve.getPoseEstimator());

    public final NoteState noteState = new NoteState(Constants.CURRENT_MODE, intake);
    public final ShootCommands shootCommands = new ShootCommands(swerve, intake, superstructure);
    public final Autos autos = new Autos(swerve, intake, superstructure, noteState, shootCommands);
    public final AutoChooser<String, AutoOption> autoChooser = new AutoChooser<>(
            new AutoOption(
                    "DoNothing",
                    autos.doNothing(),
                    Constants.CompetitionType.COMPETITION
            )
    );

    public final CommandXboxController driverController = new CommandXboxController(RobotMap.MainController);
    public final CommandXboxController coDriverController = new CommandXboxController(RobotMap.CoController);

    private EventLoop autonomousEventLoop;
    private final EventLoop teleopEventLoop = new EventLoop();
    private final EventLoop testEventLoop = new EventLoop();

    private final Trigger teleopEnabled = new Trigger(DriverStation::isTeleopEnabled);

    @Override
    public void robotInit() {
        if ((RobotBase.isReal() && Constants.CURRENT_MODE != Constants.RobotMode.REAL)
                || (RobotBase.isSimulation() && Constants.CURRENT_MODE == Constants.RobotMode.REAL)
        ) {
            DriverStation.reportWarning(String.format(
                    "Potentially incorrect CURRENT_MODE \"%s\" specified, robot is running \"%s\"",
                    Constants.CURRENT_MODE,
                    RobotBase.getRuntimeType().toString()
            ), true);

            throw new RuntimeException("Incorrect CURRENT_MODE specified!");
        }

        // we never use LiveWindow, and apparently this causes loop overruns so disable it
        LiveWindow.disableAllTelemetry();
        LiveWindow.setEnabled(false);

        // register shutdown hook
        ToClose.hook();

        // disable joystick not found warnings when in sim
        DriverStation.silenceJoystickConnectionWarning(Constants.CURRENT_MODE == Constants.RobotMode.SIM);

        // record git metadata
        Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
        Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
        Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
        Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
        Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
        // no need to inspect this here because BuildConstants is a dynamically changing file upon compilation
        //noinspection RedundantSuppression
        switch (BuildConstants.DIRTY) {
            //noinspection DataFlowIssue
            case 0 -> Logger.recordMetadata("GitDirty", "All changes committed");
            //noinspection DataFlowIssue
            case 1 -> Logger.recordMetadata("GitDirty", "Uncommitted changes");
            //noinspection DataFlowIssue
            default -> Logger.recordMetadata("GitDirty", "Unknown");
        }

        switch (Constants.CURRENT_MODE) {
            case REAL -> {
                try {
                    Files.createDirectories(Paths.get(HootLogPath));
                    SignalLogger.setPath(HootLogPath);
                } catch (final IOException ioException) {
                    SignalLogger.setPath("/U");
                    DriverStation.reportError(
                            String.format(
                                    "Failed to create CTRE .hoot log path at \"%s\"! Falling back to default.\n%s",
                                    HootLogPath,
                                    ioException
                            ),
                            false
                    );
                }

                Logger.addDataReceiver(new WPILOGWriter(AKitLogPath));
                Logger.addDataReceiver(new NT4Publisher());
            }
            case SIM -> {
                // log to working directory when running sim
                // setPath doesn't seem to work in sim (path is ignored and hoot files are always sent to /logs)
//                SignalLogger.setPath("/logs");
                Logger.addDataReceiver(new WPILOGWriter(""));
                Logger.addDataReceiver(new NT4Publisher());
            }
            case REPLAY -> {
                setUseTiming(false);
                final String logPath = LogFileUtil.findReplayLog();
                Logger.setReplaySource(new WPILOGReader(logPath));
                Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
            }
        }

        powerDistribution.clearStickyFaults();
        powerDistribution.setSwitchableChannel(true);

        configureStateTriggers();
        configureAutos();
        configureButtonBindings(teleopEventLoop);

        SignalLogger.enableAutoLogging(true);
        SignalLogger.start();
        ToClose.add(SignalLogger::stop);

        CommandScheduler.getInstance().onCommandInitialize(
                command -> Logger.recordOutput("Commands/Initialized", command.getName())
        );

        CommandScheduler.getInstance().onCommandFinish(
                command -> Logger.recordOutput("Commands/Finished", command.getName())
        );

        // TODO: don't use streams
        CommandScheduler.getInstance().onCommandInterrupt(
                (interrupted, interrupting) -> {
                    Logger.recordOutput("Commands/Interrupted", interrupted.getName());
                    Logger.recordOutput(
                            "Commands/InterruptedRequirements",
                            interrupted.getRequirements()
                                    .stream()
                                    .map(Subsystem::getName)
                                    .toArray(String[]::new)
                    );

                    Logger.recordOutput("Commands/Interrupting", interrupting.isPresent()
                            ? interrupting.get().getName()
                            : "None"
                    );
                    Logger.recordOutput(
                            "Commands/InterruptingRequirements",
                            interrupting
                                    .map(command -> command.getRequirements()
                                            .stream()
                                            .map(Subsystem::getName)
                                            .toArray(String[]::new)
                                    ).orElseGet(() -> new String[0])
                    );
                }
        );

        Logger.start();
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        VirtualSubsystem.run();

        final double distanceToSpeaker = swerve.getPose()
                .minus(FieldConstants.getSpeakerPose())
                .getTranslation()
                .getNorm();
        final ShotParameters.Parameters shotParameters = ShotParameters.get(distanceToSpeaker);

        Logger.recordOutput("ShotParameters/SpeakerDistance", distanceToSpeaker);
        Logger.recordOutput("ShotParameters/ArmPivotAngle", shotParameters.armPivotAngle().getRotations());
        Logger.recordOutput("ShotParameters/LeftVelocityRotsPerSec", shotParameters.leftVelocityRotsPerSec());
        Logger.recordOutput("ShotParameters/RightVelocityRotsPerSec", shotParameters.rightVelocityRotsPerSec());
        Logger.recordOutput("ShotParameters/AmpVelocityRotsPerSec", shotParameters.ampVelocityRotsPerSec());

        final ShootOnTheMove.Shot shotWhileMoving = ShootOnTheMove.calculate(
                swerve.getPose(),
                swerve.getRobotRelativeSpeeds(),
                currentPose -> shotParameters
        );

        Logger.recordOutput("ShootWhileMoving/FuturePose", shotWhileMoving.futurePose());

        final NoteState.State state = noteState.getState();
        Logger.recordOutput("NoteState", state.toString());
    }

    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {}

    @Override
    public void autonomousInit() {
        autonomousEventLoop = autoChooser.getSelected().autoEventLoop();
    }

    @Override
    public void autonomousPeriodic() {
        if (autonomousEventLoop != null) {
            autonomousEventLoop.poll();
        }
    }

    @Override
    public void teleopInit() {
        //noinspection SuspiciousNameCombination
        swerve.setDefaultCommand(
                swerve.teleopDriveCommand(
                        driverController::getLeftY,
                        driverController::getLeftX,
                        driverController::getRightX
                )
        );
    }

    @Override
    public void teleopPeriodic() {
        if (teleopEventLoop != null) {
            teleopEventLoop.poll();
        }
    }

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();

        coDriverController.leftBumper(testEventLoop)
                .onTrue(Commands.runOnce(SignalLogger::stop));

        coDriverController.y(testEventLoop).and(coDriverController.rightBumper(testEventLoop))
                .whileTrue(shooter.torqueCurrentSysIdCommand());

        coDriverController.x(testEventLoop).and(coDriverController.rightBumper(testEventLoop))
                .whileTrue(arm.voltageSysIdCommand());

        coDriverController.a(testEventLoop).and(coDriverController.rightBumper(testEventLoop))
                .whileTrue(intake.torqueCurrentSysIdCommand());

//        coDriverController.y(testEventLoop)
//                .whileTrue(swerve
//                        .angularVoltageSysIdQuasistaticCommand(SysIdRoutine.Direction.kForward));
//        coDriverController.a(testEventLoop)
//                .whileTrue(swerve
//                        .angularVoltageSysIdQuasistaticCommand(SysIdRoutine.Direction.kReverse));
//        coDriverController.b(testEventLoop)
//                .whileTrue(swerve
//                        .angularVoltageSysIdDynamicCommand(SysIdRoutine.Direction.kForward));
//        coDriverController.x(testEventLoop)
//                .whileTrue(swerve
//                        .angularVoltageSysIdDynamicCommand(SysIdRoutine.Direction.kReverse));
    }

    @Override
    public void testPeriodic() {
        if (testEventLoop != null) {
            testEventLoop.poll();
        }
    }

    @Override
    public void simulationInit() {}

    @Override
    public void simulationPeriodic() {}

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

    public void configureStateTriggers() {
        teleopEnabled.onTrue(superstructure.toGoal(Superstructure.Goal.IDLE));
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
        autoChooser.addAutoOption(new AutoOption(
                "Speaker2_1_0Center4_3",
                autos.speaker2_1_0Center4_3(),
                Constants.CompetitionType.COMPETITION
        ));
    }

    public void configureButtonBindings(final EventLoop teleopEventLoop) {
        final XboxController driverHID = driverController.getHID();
        this.driverController.leftTrigger(0.5, teleopEventLoop).whileTrue(intake.intakeCommand());
        // TODO: does this rumble fast/early enough?
        this.noteState.hasNote.onTrue(
                ControllerUtils.rumbleForDurationCommand(
                        driverController.getHID(),
                        GenericHID.RumbleType.kBothRumble,
                        0.5,
                        0.5
                )
        );

        this.driverController.rightTrigger(0.5, teleopEventLoop)
                .whileTrue(shootCommands.deferredStopAimAndShoot())
//                .whileTrue(shootCommands.teleopDriveAimAndShoot(
//                        driverController::getLeftY,
//                        driverController::getLeftX
//                ))
                .onFalse(superstructure.toGoal(Superstructure.Goal.IDLE));

        this.driverController.a(teleopEventLoop).whileTrue(shootCommands.amp());
        this.driverController.y(teleopEventLoop).onTrue(swerve.zeroRotationCommand());

        this.driverController.b(teleopEventLoop).whileTrue(intake.feedCommand());

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
}
