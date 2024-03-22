package frc.robot;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.constants.Constants;
import frc.robot.constants.FieldConstants;
import frc.robot.state.NoteState;
import frc.robot.subsystems.superstructure.ShotParameters;
import frc.robot.utils.closeables.ToClose;
import frc.robot.utils.subsystems.VirtualSubsystem;
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

    private RobotContainer robotContainer;

    private EventLoop autonomousEventLoop;
    private final EventLoop teleopEventLoop = new EventLoop();
    private final EventLoop testEventLoop = new EventLoop();

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

        robotContainer = new RobotContainer();
        robotContainer.configureAutos();
        robotContainer.configureButtonBindings(teleopEventLoop);

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

        final double distanceToSpeaker = robotContainer.swerve.getPose()
                .minus(FieldConstants.getSpeakerPose())
                .getTranslation()
                .getNorm();
        final ShotParameters.Parameters shotParameters = ShotParameters.get(distanceToSpeaker);

        Logger.recordOutput("ShotParameters/SpeakerDistance", distanceToSpeaker);
        Logger.recordOutput("ShotParameters/ArmPivotAngle", shotParameters.armPivotAngle().getRotations());
        Logger.recordOutput("ShotParameters/LeftVelocityRotsPerSec", shotParameters.leftVelocityRotsPerSec());
        Logger.recordOutput("ShotParameters/RightVelocityRotsPerSec", shotParameters.rightVelocityRotsPerSec());
        Logger.recordOutput("ShotParameters/AmpVelocityRotsPerSec", shotParameters.ampVelocityRotsPerSec());

        final NoteState.State state = robotContainer.noteState.getState();
        Logger.recordOutput("NoteState", state.toString());
    }

    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {}

    @Override
    public void autonomousInit() {
        autonomousEventLoop = robotContainer.getAutonomousEventLoop();
    }

    @Override
    public void autonomousPeriodic() {
        if (autonomousEventLoop != null) {
            autonomousEventLoop.poll();
        }
    }

    @Override
    public void teleopInit() {
        // TODO: superstructure to IDLE state on teleop init
        // TODO: bonus points if it homes/zeros itself if it isn't already zeroed,
        //  although, if we have an absolute encoder on it, we won't need to zero anymore
//        if (!robotContainer.superstructure.isHomed()) {
//            // TODO: do this better
//            Commands.sequence(
//                    robotContainer.superstructure.home(),
//                    robotContainer.superstructure.toGoal(Superstructure.Goal.IDLE)
//            ).schedule();
//        } else {
//            robotContainer.superstructure.toGoal(Superstructure.Goal.IDLE).schedule();
//        }

        //noinspection SuspiciousNameCombination
        robotContainer.swerve.setDefaultCommand(
                robotContainer.swerve.teleopDriveCommand(
                        robotContainer.driverController::getLeftY,
                        robotContainer.driverController::getLeftX,
                        robotContainer.driverController::getRightX
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

        robotContainer.coDriverController.leftBumper(testEventLoop)
                .onTrue(Commands.runOnce(SignalLogger::stop));

        robotContainer.coDriverController.y(testEventLoop)
                .whileTrue(robotContainer.swerve
                        .angularVoltageSysIdQuasistaticCommand(SysIdRoutine.Direction.kForward));
        robotContainer.coDriverController.a(testEventLoop)
                .whileTrue(robotContainer.swerve
                        .angularVoltageSysIdQuasistaticCommand(SysIdRoutine.Direction.kReverse));
        robotContainer.coDriverController.b(testEventLoop)
                .whileTrue(robotContainer.swerve
                        .angularVoltageSysIdDynamicCommand(SysIdRoutine.Direction.kForward));
        robotContainer.coDriverController.x(testEventLoop)
                .whileTrue(robotContainer.swerve
                        .angularVoltageSysIdDynamicCommand(SysIdRoutine.Direction.kReverse));
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
}
