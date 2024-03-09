package frc.robot.auto;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;
import com.choreo.lib.ChoreoTrajectoryState;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.drive.Swerve;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.superstructure.ShotParameters;
import frc.robot.subsystems.superstructure.Superstructure;

import java.util.List;
import java.util.Optional;
import java.util.Set;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class Autos {
    private static final double TranslationToleranceMeters = 0.1;
    private static final double TimeToleranceSeconds = 0.1;

    private static final BooleanSupplier IsRedAlliance = () -> {
        final Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
        return alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;
    };

    private final Swerve swerve;
    private final Superstructure superstructure;
    private final Intake intake;

    public Autos(final Swerve swerve, final Superstructure superstructure, final Intake intake) {
        this.swerve = swerve;
        this.superstructure = superstructure;
        this.intake = intake;
    }

    private static class AutoTriggers {
        private final ChoreoTrajectory choreoTrajectory;
        private final Supplier<Pose2d> poseSupplier;
        private final DoubleSupplier timeSupplier;
        private final EventLoop eventLoop;

        public AutoTriggers(
                final ChoreoTrajectory choreoTrajectory,
                final Supplier<Pose2d> poseSupplier,
                final DoubleSupplier timeSupplier
        ) {
            this.choreoTrajectory = choreoTrajectory;
            this.poseSupplier = poseSupplier;
            this.timeSupplier = timeSupplier;
            this.eventLoop = new EventLoop();
        }

        public void poll() {
            eventLoop.poll();
        }

        public Trigger autoEnabled() {
            return new Trigger(eventLoop, DriverStation::isAutonomousEnabled);
        }

        //TODO: Fix this it doesn't work
        public Trigger atPlaceAndTime(final double timeSeconds) {
            final Translation2d place = choreoTrajectory
                    .sample(timeSeconds, IsRedAlliance.getAsBoolean())
                    .getPose()
                    .getTranslation();
            return new Trigger(
                    eventLoop,
                    () -> poseSupplier
                            .get()
                            .getTranslation()
                            .getDistance(place) < TranslationToleranceMeters
                            && MathUtil.isNear(timeSeconds, timeSupplier.getAsDouble(), TimeToleranceSeconds)
            );
        }

        public Trigger atTime(final double timeSeconds) {
            return new Trigger(
                    eventLoop,
                    () -> MathUtil.isNear(timeSeconds, timeSupplier.getAsDouble(), TimeToleranceSeconds)
            );
        }
        
        //TODO: Fix this it doesn't work
        public Trigger atPlace(final double timeSeconds) {
            final Translation2d place = choreoTrajectory
                    .sample(timeSeconds, IsRedAlliance.getAsBoolean())
                    .getPose()
                    .getTranslation();
            return new Trigger(
                    eventLoop,
                    () -> poseSupplier
                            .get()
                            .getTranslation()
                            .getDistance(place) < TranslationToleranceMeters
            );
        }
    }

    private Command followPath(final ChoreoTrajectory choreoTrajectory, final Timer timer) {
        return Commands.sequence(
                Commands.runOnce(timer::start),
                swerve.followChoreoPathCommand(choreoTrajectory),
                Commands.runOnce(timer::stop)
        );
    }

    private Command shoot() {
        return Commands.deadline(
                intake
                        .runStopCommand()
                        .until(superstructure.atGoalTrigger)
                        .andThen(intake.feedCommand()),
                superstructure.toGoal(Superstructure.Goal.SUBWOOFER)
        );
    }

    private Command initAndPoll(final Command initCommand, final Runnable poll) {
        return Commands.parallel(
                initCommand,
                Commands.run(poll)
                        .until(() -> !DriverStation.isAutonomousEnabled())
        );
    }
    
    public Command sourceSpeaker0() {
        final String trajectoryName = "SourceSpeaker0";
        final ChoreoTrajectory trajectory = Choreo.getTrajectory(trajectoryName);
        final List<ChoreoTrajectory> trajectoryGroup = Choreo.getTrajectoryGroup(trajectoryName);

        final Timer timer = new Timer();
        final AutoTriggers autoTriggers = new AutoTriggers(trajectory, swerve::getPose, timer::get);

        final ChoreoTrajectoryState initialState = trajectory.getInitialState();
        final Command autoInitCommand = Commands.sequence(
                Commands.defer(() -> swerve.resetPoseCommand(
                                IsRedAlliance.getAsBoolean()
                                        ? initialState.flipped().getPose()
                                        : initialState.getPose()
                        ),
                        Set.of(swerve)
                ),
                Commands.print("shoot preload"),
//                shoot(),
                followPath(trajectoryGroup.get(0), timer)
        );

        autoTriggers.autoEnabled().whileTrue(autoInitCommand);

        autoTriggers.atTime(0.1).onTrue(
                Commands.sequence(
                        Commands.print("run intake"),
                        intake.intakeCommand()
                )
        );

        autoTriggers.atTime(1.3).onTrue(
                Commands.sequence(
                        Commands.print("Shoot"),
                        shoot()
                )
        );

        return initAndPoll(autoInitCommand, autoTriggers::poll);
    }

    public Command sourceSpeaker0Center1() {
        final String trajectoryName = "SourceSpeaker0Center1";
        final ChoreoTrajectory trajectory = Choreo.getTrajectory(trajectoryName);
        final List<ChoreoTrajectory> trajectoryGroup = Choreo.getTrajectoryGroup(trajectoryName);

        final Timer timer = new Timer();
        final AutoTriggers autoTriggers = new AutoTriggers(trajectory, swerve::getPose, timer::get);

        final ChoreoTrajectoryState initialState = trajectory.getInitialState();
        autoTriggers.autoEnabled().whileTrue(
                Commands.sequence(
                        Commands.defer(() -> swerve.resetPoseCommand(
                                        IsRedAlliance.getAsBoolean()
                                                ? initialState.flipped().getPose()
                                                : initialState.getPose()
                                ),
                                Set.of(swerve)
                        ),
                        Commands.print("shoot preload"),
                        shoot(),
                        followPath(trajectoryGroup.get(0), timer)
                )

        );

        autoTriggers.atTime(0.1).onTrue(
                Commands.sequence(
                        Commands.print("run intake"),
                        intake.intakeCommand()
                )
        );

        autoTriggers.atTime(1.3).onTrue(
                Commands.sequence(
                        Commands.print("Shoot"),
                        shoot(),
                        followPath(trajectoryGroup.get(1), timer)
                )
        );

        autoTriggers.atTime(2.2).onTrue(
                Commands.sequence(
                        Commands.print("run intake"),
                        intake.intakeCommand()
                )
        );

        autoTriggers.atTime(5.81).onTrue(
                Commands.sequence(
                        Commands.print("shoot"),
                        shoot()
                )
        );

        return Commands.run(autoTriggers::poll)
                .until(() -> !DriverStation.isAutonomousEnabled());
    }

    public Command sourceSpeaker0Center1_2() {
        final String trajectoryName = "SourceSpeaker0Center1_2";
        final ChoreoTrajectory trajectory = Choreo.getTrajectory(trajectoryName);
        final List<ChoreoTrajectory> trajectoryGroup = Choreo.getTrajectoryGroup(trajectoryName);

        final Timer timer = new Timer();
        final AutoTriggers autoTriggers = new AutoTriggers(trajectory, swerve::getPose, timer::get);

        final ChoreoTrajectoryState initialState = trajectory.getInitialState();

        autoTriggers.autoEnabled().whileTrue(
                Commands.sequence(
                        Commands.defer(() -> swerve.resetPoseCommand(
                                        IsRedAlliance.getAsBoolean()
                                                ? initialState.flipped().getPose()
                                                : initialState.getPose()
                                ),
                                Set.of(swerve)
                        ),
                        Commands.print("shoot preload"),
                        shoot(),
                        followPath(trajectoryGroup.get(0), timer)
                )

        );

        autoTriggers.atTime(0.1).onTrue(
                Commands.sequence(
                        Commands.print("run intake"),
                        intake.intakeCommand()
                )
        );

        autoTriggers.atTime(1.3).onTrue(
                Commands.sequence(
                        Commands.print("Shoot"),
                        shoot(),
                        followPath(trajectoryGroup.get(1), timer)
                )
        );

        autoTriggers.atTime(2.2).onTrue(
                Commands.sequence(
                        Commands.print("run intake"),
                        intake.intakeCommand()
                )
        );

        autoTriggers.atTime(5.81).onTrue(
                Commands.sequence(
                        Commands.print("shoot"),
                        shoot(),
                        followPath(trajectoryGroup.get(2), timer)
                )
        );

        autoTriggers.atTime(7).onTrue(
                Commands.sequence(
                        Commands.print("run intake"),
                        intake.intakeCommand()
                )
        );

        autoTriggers.atTime(10.4).onTrue(
                Commands.sequence(
                        Commands.print("shoot"),
                        shoot()
                )
        );

        return Commands.run(autoTriggers::poll)
                .until(() -> !DriverStation.isAutonomousEnabled());
    }

    public Command walton() {
        final String trajectoryName = "Walton";
        final ChoreoTrajectory trajectory = Choreo.getTrajectory(trajectoryName);
        final List<ChoreoTrajectory> trajectoryGroup = Choreo.getTrajectoryGroup(trajectoryName);

        final Timer timer = new Timer();
        final AutoTriggers autoTriggers = new AutoTriggers(trajectory, swerve::getPose, timer::get);

        final ChoreoTrajectoryState initialState = trajectory.getInitialState();
        autoTriggers.autoEnabled().whileTrue(
                Commands.sequence(
                        Commands.defer(() -> swerve.resetPoseCommand(
                                        IsRedAlliance.getAsBoolean()
                                                ? initialState.flipped().getPose()
                                                : initialState.getPose()
                                ),
                                Set.of(swerve)
                        )  ,
                        Commands.print("shoot preload"),
                        shoot(),
                        followPath(trajectoryGroup.get(0), timer)
                )
        );

        return Commands.run(autoTriggers::poll)
                .until(() -> !DriverStation.isAutonomousEnabled());
    }

    public Command speaker0_1_2() {
        final String trajectoryName = "Speaker0_1_2";
        final ChoreoTrajectory trajectory = Choreo.getTrajectory(trajectoryName);
        final List<ChoreoTrajectory> trajectoryGroup = Choreo.getTrajectoryGroup(trajectoryName);

        final Timer timer = new Timer();
        final AutoTriggers autoTriggers = new AutoTriggers(trajectory, swerve::getPose, timer::get);

        final ChoreoTrajectoryState initialState = trajectory.getInitialState();
        autoTriggers.autoEnabled().whileTrue(
                Commands.sequence(
                        Commands.defer(() -> swerve.resetPoseCommand(
                                        IsRedAlliance.getAsBoolean()
                                                ? initialState.flipped().getPose()
                                                : initialState.getPose()
                                ),
                                Set.of(swerve)
                        ),
                        Commands.print("shoot preload"),
                        shoot(),
                        followPath(trajectoryGroup.get(0), timer)
                )

        );

        autoTriggers.atTime(0.1).onTrue(
                Commands.sequence(
                        Commands.print("run intake"),
                        intake.intakeCommand()
                )
        );

        autoTriggers.atTime(1.58).onTrue(
                Commands.sequence(
                        Commands.print("Shoot"),
                        shoot(),
                        followPath(trajectoryGroup.get(1), timer)
                )
        );

        autoTriggers.atTime(1.7).onTrue(
                Commands.sequence(
                        Commands.print("run intake"),
                        intake.intakeCommand()
                )
        );

        autoTriggers.atTime(2.92).onTrue(
                Commands.sequence(
                        Commands.print("shoot"),
                        shoot(),
                        followPath(trajectoryGroup.get(2), timer)
                )
        );

        autoTriggers.atTime(3).onTrue(
                Commands.sequence(
                        Commands.print("run intake"),
                        intake.intakeCommand()
                )
        );

        autoTriggers.atTime(4.31).onTrue(
                Commands.sequence(
                        Commands.print("shoot"),
                        shoot()
                )
        );

        return Commands.run(autoTriggers::poll)
                .until(() -> !DriverStation.isAutonomousEnabled());
    }

    public Command ampSpeaker2Center3() {
        final String trajectoryName = "AmpSpeaker2Center3";
        final ChoreoTrajectory trajectory = Choreo.getTrajectory(trajectoryName);
        final List<ChoreoTrajectory> trajectoryGroup = Choreo.getTrajectoryGroup(trajectoryName);

        final Timer timer = new Timer();
        final AutoTriggers autoTriggers = new AutoTriggers(trajectory, swerve::getPose, timer::get);

        final ChoreoTrajectoryState initialState = trajectory.getInitialState();
        autoTriggers.autoEnabled().whileTrue(
                Commands.sequence(
                        Commands.defer(() -> swerve.resetPoseCommand(
                                        IsRedAlliance.getAsBoolean()
                                                ? initialState.flipped().getPose()
                                                : initialState.getPose()
                                ),
                                Set.of(swerve)
                        ),
                        Commands.print("shoot preload"),
                        shoot(),
                        followPath(trajectoryGroup.get(0), timer)
                )

        );

        autoTriggers.atTime(0.1).onTrue(
                Commands.sequence(
                        Commands.print("run intake"),
                        intake.intakeCommand()
                )
        );

        autoTriggers.atTime(1.46).onTrue(
                Commands.sequence(
                        Commands.print("Shoot"),
                        shoot(),
                        followPath(trajectoryGroup.get(1), timer)
                )
        );

        autoTriggers.atTime(1.6).onTrue(
                Commands.sequence(
                        Commands.print("run intake"),
                        intake.intakeCommand()
                )
        );

        autoTriggers.atTime(5.59).onTrue(
                Commands.sequence(
                        Commands.print("shoot"),
                        shoot()
                )
        );

        return Commands.run(autoTriggers::poll)
                .until(() -> !DriverStation.isAutonomousEnabled());
    }

    public Command shootAndMobility() {
        final String trajectoryName = "ShootAndMobility";
        final ChoreoTrajectory trajectory = Choreo.getTrajectory(trajectoryName);
        final List<ChoreoTrajectory> trajectoryGroup = Choreo.getTrajectoryGroup(trajectoryName);

        final Timer timer = new Timer();
        final AutoTriggers autoTriggers = new AutoTriggers(trajectory, swerve::getPose, timer::get);

        final ChoreoTrajectoryState initialState = trajectory.getInitialState();
        final Command autoInitCommand = Commands.sequence(
                Commands.defer(() -> swerve.resetPoseCommand(
                                IsRedAlliance.getAsBoolean()
                                        ? initialState.flipped().getPose()
                                        : initialState.getPose()
                        ),
                        Set.of(swerve)
                ),
                Commands.print("shoot preload"),
                shoot(),
                followPath(trajectoryGroup.get(0), timer)
        );

        autoTriggers.autoEnabled()
                .whileTrue(autoInitCommand)
                .onFalse(Commands.runOnce(swerve::stop));

        return initAndPoll(autoInitCommand, autoTriggers::poll);
    }
}
