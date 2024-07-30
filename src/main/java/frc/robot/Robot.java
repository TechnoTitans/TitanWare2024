package frc.robot;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
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
import frc.robot.utils.logging.LogUtils;
import frc.robot.utils.subsystems.VirtualSubsystem;
import frc.robot.utils.teleop.ControllerUtils;
import frc.robot.utils.teleop.Profiler;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.util.Optional;
import java.util.Set;
import java.util.function.BooleanSupplier;

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
    public final ShootCommands shootCommands = new ShootCommands(swerve, intake, superstructure, noteState);
    public final Autos autos = new Autos(swerve, intake, superstructure, photonVision, noteState, shootCommands);
    public final AutoChooser<String, AutoOption> autoChooser = new AutoChooser<>(
            new AutoOption(
                    "DoNothing",
                    autos.driveAndNoteDetect(),
                    Constants.CompetitionType.COMPETITION
            )
    );

    public final CommandXboxController driverController = new CommandXboxController(RobotMap.MainController);
    public final CommandXboxController coDriverController = new CommandXboxController(RobotMap.CoController);

    private EventLoop autonomousEventLoop;
    private final EventLoop teleopEventLoop = new EventLoop();
    private final EventLoop testEventLoop = new EventLoop();

    private final Trigger autoEnabled = new Trigger(DriverStation::isAutonomousEnabled);
    private final Trigger teleopEnabled = new Trigger(DriverStation::isTeleopEnabled);
    private final Trigger endgameTrigger = new Trigger(
            () -> DriverStation.getMatchTime() <= 20
                    && DriverStation.isFMSAttached()
    ).and(teleopEnabled);

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
            //noinspection DataFlowIssue,RedundantSuppression
            case 0 -> //noinspection UnreachableCode
                    Logger.recordMetadata("GitDirty", "All changes committed");
            //noinspection DataFlowIssue,RedundantSuppression
            case 1 -> //noinspection UnreachableCode
                    Logger.recordMetadata("GitDirty", "Uncommitted changes");
            //noinspection DataFlowIssue,RedundantSuppression
            default -> //noinspection UnreachableCode
                    Logger.recordMetadata("GitDirty", "Unknown");
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
                                    "Failed to create .hoot log path at \"%s\"! Falling back to default.\n%s",
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
                // Disable Protobuf log overhead warning in replay
                LogTable.disableProtobufWarning();
                setUseTiming(false);

                final String logPath = LogFileUtil.findReplayLog();
                Logger.setReplaySource(new WPILOGReader(logPath));
                Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim"), 0.005));
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

        CommandScheduler.getInstance().onCommandInterrupt(
                (interrupted, interrupting) -> {
                    Logger.recordOutput("Commands/Interrupted", interrupted.getName());

                    Logger.recordOutput(
                            "Commands/InterruptedRequirements",
                            LogUtils.getRequirementsFromSubsystems(interrupted.getRequirements())
                    );

                    Logger.recordOutput("Commands/Interrupting", interrupting.isPresent()
                            ? interrupting.get().getName()
                            : "None"
                    );

                    Logger.recordOutput(
                            "Commands/InterruptingRequirements",
                            LogUtils.getRequirementsFromSubsystems(
                                    interrupting.isPresent() ? interrupting.get().getRequirements() : Set.of()
                            )
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
        final ShotParameters.Parameters shotParameters = ShotParameters.getShotParameters(swerve.getPose());

        Logger.recordOutput("ShotParameters/SpeakerDistance", distanceToSpeaker);
        Logger.recordOutput("ShotParameters/ArmPivotAngle", shotParameters.armPivotAngle().getRotations());
        Logger.recordOutput("ShotParameters/LeftVelocityRotsPerSec", shotParameters.leftVelocityRotsPerSec());
        Logger.recordOutput("ShotParameters/RightVelocityRotsPerSec", shotParameters.rightVelocityRotsPerSec());
        Logger.recordOutput("ShotParameters/AmpVelocityRotsPerSec", shotParameters.ampVelocityRotsPerSec());

        final double distanceToFerry = swerve.getPose()
                .minus(FieldConstants.getFerryPose())
                .getTranslation()
                .getNorm();
        final ShotParameters.Parameters ferryParameters = ShotParameters.getFerryParameters(swerve.getPose());

        Logger.recordOutput("FerryParameters/FerryDistance", distanceToFerry);
        Logger.recordOutput("FerryParameters/ArmPivotAngle", ferryParameters.armPivotAngle().getRotations());
        Logger.recordOutput("FerryParameters/LeftVelocityRotsPerSec", ferryParameters.leftVelocityRotsPerSec());
        Logger.recordOutput("FerryParameters/RightVelocityRotsPerSec", ferryParameters.rightVelocityRotsPerSec());
        Logger.recordOutput("FerryParameters/AmpVelocityRotsPerSec", ferryParameters.ampVelocityRotsPerSec());

        final ShootOnTheMove.Shot shotWhileMoving = ShootOnTheMove.calculate(
                swerve.getPose(),
                swerve.getRobotRelativeSpeeds(),
                currentPose -> shotParameters
        );

        Logger.recordOutput("ShootWhileMoving/FuturePose", shotWhileMoving.futurePose());
    }

    @Override
    public void disabledPeriodic() {}

    @Override
    public void autonomousInit() {
        autonomousEventLoop = autoChooser.getSelected().autoEventLoop();
        autoEnabled.onFalse(Commands.runOnce(() -> {
            if (autonomousEventLoop != null) {
                autonomousEventLoop.poll();
            }
        }).ignoringDisable(true));
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
                        driverController::getRightX,
                        IsRedAlliance
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

        coDriverController.y(testEventLoop)
                .whileTrue(swerve.linearTorqueCurrentSysIdQuasistaticCommand(SysIdRoutine.Direction.kForward));
        coDriverController.a(testEventLoop)
                .whileTrue(swerve.linearTorqueCurrentSysIdQuasistaticCommand(SysIdRoutine.Direction.kReverse));
        coDriverController.b(testEventLoop)
                .whileTrue(swerve.linearTorqueCurrentSysIdDynamicCommand(SysIdRoutine.Direction.kForward));
        coDriverController.x(testEventLoop)
                .whileTrue(swerve.linearTorqueCurrentSysIdDynamicCommand(SysIdRoutine.Direction.kReverse));
    }

    @Override
    public void testPeriodic() {
        if (testEventLoop != null) {
            testEventLoop.poll();
        }
    }

    @Override
    public void simulationPeriodic() {}

    public void configureStateTriggers() {
        teleopEnabled.onTrue(superstructure.toInstantGoal(Superstructure.Goal.IDLE));

        endgameTrigger.onTrue(ControllerUtils.rumbleForDurationCommand(
                coDriverController.getHID(), GenericHID.RumbleType.kBothRumble, 0.5, 1)
        );
    }

    public void configureAutos() {
        autoChooser.addAutoOption(new AutoOption(
                "Speaker2_1_0",
                autos.speaker2_1_0(),
                Constants.CompetitionType.COMPETITION
        ));
        autoChooser.addAutoOption(new AutoOption(
                "Speaker0_1_2",
                autos.speaker0_1_2(),
                Constants.CompetitionType.COMPETITION
        ));
        autoChooser.addAutoOption(new AutoOption(
                "Speaker0_1_2Center4_3",
                autos.speaker0_1_2Center4_3(),
                Constants.CompetitionType.TESTING
        ));
        autoChooser.addAutoOption(new AutoOption(
                "Speaker0_1_2Center4_3_2",
                autos.speaker0_1_2Center4_3_2(),
                Constants.CompetitionType.TESTING
        ));

        autoChooser.addAutoOption(new AutoOption(
                "AmpCenter3_2",
                autos.ampCenter3_2(),
                Constants.CompetitionType.COMPETITION
        ));
        autoChooser.addAutoOption(new AutoOption(
                "AmpCenter2_3",
                autos.ampCenter2_3(),
                Constants.CompetitionType.COMPETITION
        ));
        autoChooser.addAutoOption(new AutoOption(
                "AmpCenter4_3",
                autos.ampCenter4_3(),
                Constants.CompetitionType.COMPETITION
        ));
        autoChooser.addAutoOption(new AutoOption(
                "AmpCenter3_4",
                autos.ampCenter3_4(),
                Constants.CompetitionType.COMPETITION
        ));
        autoChooser.addAutoOption(new AutoOption(
                "AmpCenter4_3_2",
                autos.ampCenter4_3_2(),
                Constants.CompetitionType.COMPETITION
        ));
        autoChooser.addAutoOption(new AutoOption(
                "AmpCenter4_2_1",
                autos.ampCenter4_2_1(),
                Constants.CompetitionType.COMPETITION
        ));
        autoChooser.addAutoOption(new AutoOption(
                "AmpSpeaker2",
                autos.ampSpeaker2(),
                Constants.CompetitionType.TESTING
        ));
        autoChooser.addAutoOption(new AutoOption(
                "AmpSpeaker2Center2_3",
                autos.ampSpeaker2Center2_3(),
                Constants.CompetitionType.TESTING
        ));
        autoChooser.addAutoOption(new AutoOption(
                "Amp2_1Center2_3_4",
                autos.amp2_1Center2_3_4(),
                Constants.CompetitionType.TESTING
        ));

        autoChooser.addAutoOption(new AutoOption(
                "SourceCenter1_0",
                autos.sourceCenter1_0(),
                Constants.CompetitionType.TESTING
        ));
        autoChooser.addAutoOption(new AutoOption(
                "SourceCenter0_1_2",
                autos.altSourceCenter0_1_2(),
                Constants.CompetitionType.COMPETITION
        ));
        autoChooser.addAutoOption(new AutoOption(
                "old_SourceCenter0_1_2",
                autos.sourceCenter0_1_2(),
                Constants.CompetitionType.TESTING
        ));
        autoChooser.addAutoOption(new AutoOption(
                "SourceCenter1_0_2",
                autos.sourceCenter1_0_2(),
                Constants.CompetitionType.COMPETITION
        ));
        autoChooser.addAutoOption(new AutoOption(
                "SourceSpeaker0",
                autos.sourceSpeaker0(),
                Constants.CompetitionType.TESTING
        ));
        autoChooser.addAutoOption(new AutoOption(
                "SourceMobility",
                autos.sourceMobility(),
                Constants.CompetitionType.TESTING
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
                "FollowNote",
                autos.followNote(),
                Constants.CompetitionType.TESTING
        ));
    }

    @SuppressWarnings("RedundantSuppression")
    public void configureButtonBindings(final EventLoop teleopEventLoop) {
        this.driverController.leftTrigger(0.5, teleopEventLoop)
                .whileTrue(intake.intakeCommand());
        // TODO: this doesn't rumble early enough, or as early as we'd like it to
        //  not sure if we're hardware limited or its behind by a few cycles and we can speed it up
        this.noteState.hasNote.onTrue(
                ControllerUtils.rumbleForDurationCommand(
                        driverController.getHID(),
                        GenericHID.RumbleType.kBothRumble,
                        0.5,
                        0.5
                )
        );

        //noinspection SuspiciousNameCombination
        this.driverController.rightTrigger(0.5, teleopEventLoop)
                .whileTrue(shootCommands.readyShot(
                        driverController::getLeftY,
                        driverController::getLeftX
                ))
                .onFalse(shootCommands.deferredStopAimAndShoot());
//                .whileTrue(shootCommands.deferredStopAimAndShoot());
//                .whileTrue(shootCommands.teleopDriveAimAndShoot(
//                        driverController::getLeftY,
//                        driverController::getLeftX
//                ));

//        this.driverController.a(teleopEventLoop)
//                .whileTrue(shootCommands.readyAmp())
//                .onFalse(shootCommands.amp());
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

        this.coDriverController.y(teleopEventLoop)
                .whileTrue(shootCommands.runEjectShooter());
        this.coDriverController.a(teleopEventLoop)
                .whileTrue(shootCommands.runEjectIntake());
        //noinspection SuspiciousNameCombination
        this.coDriverController.b(teleopEventLoop)
                .whileTrue(shootCommands.angleAndReadyAmp(
                        driverController::getLeftY,
                        driverController::getLeftX
                )).onFalse(shootCommands.amp());
//                .whileTrue(shootCommands.lineupAndAmp());
        //noinspection SuspiciousNameCombination
        this.coDriverController.leftTrigger(0.5, teleopEventLoop)
                .whileTrue(shootCommands.readyFerry(
                        driverController::getLeftY,
                        driverController::getLeftX
                ))
                .onFalse(shootCommands.ferry());

        this.coDriverController.rightTrigger(0.5, teleopEventLoop)
                .whileTrue(shootCommands.shootSubwoofer());
    }
}
