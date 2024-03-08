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
import frc.robot.subsystems.drive.Swerve;

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

    public Autos(final Swerve swerve) {
        this.swerve = swerve;
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

    
    public Command sourceSpeaker0() {
        final String trajectoryName = "SourceSpeaker0";
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
                        Commands.waitUntil(() -> true), // wait until shot goes out
                        followPath(trajectoryGroup.get(0), timer)
                )
        );

        autoTriggers.atTime(0.1).onTrue(
                Commands.sequence(
                        Commands.print("run intake for close 0 (first note)"),
                        Commands.waitUntil(() -> true).withTimeout(2), // wait until intake has note, or timeout
                        Commands.print("run intake to idle")
                )
        );

        autoTriggers.atTime(1.3).onTrue(
                Commands.sequence(
                        Commands.print("Shoot"),
                        Commands.waitUntil(() -> true)
                )
        );

        return Commands.run(autoTriggers::poll)
                .until(() -> !DriverStation.isAutonomousEnabled());
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
                        Commands.waitUntil(() -> true), // wait until shot goes out
                        followPath(trajectoryGroup.get(0), timer)
                )

        );

        autoTriggers.atTime(0.1).onTrue(
                Commands.sequence(
                        Commands.print("run intake for close 0 (first note)"),
                        Commands.waitUntil(() -> true).withTimeout(2), // wait until intake has note, or timeout
                        Commands.print("run intake to idle")
                )
        );

        autoTriggers.atTime(1.3).onTrue(
                Commands.sequence(
                        Commands.print("Shoot"),
                        Commands.waitUntil(() -> true),
                        followPath(trajectoryGroup.get(1), timer)
                )
        );

        autoTriggers.atTime(2.2).onTrue(
                Commands.sequence(
                        Commands.print("run intake for close 0 (first note)"),
                        Commands.waitUntil(() -> true).withTimeout(4), // wait until intake has note, or timeout
                        Commands.print("run intake to idle")
                )
        );

        autoTriggers.atTime(5.81).onTrue(
                Commands.sequence(
                        Commands.print("shoot"),
                        Commands.waitUntil(() -> true).withTimeout(1)
                )
        );

        return Commands.run(autoTriggers::poll)
                .until(() -> !DriverStation.isAutonomousEnabled());
    }

    public Command sourceSpeaker0Center1_2() {
        final String trajectoryName = "SourceSpeaker0Center1_2.1";
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
                        Commands.waitUntil(() -> true), // wait until shot goes out
                        followPath(trajectoryGroup.get(0), timer)
                )

        );

        autoTriggers.atTime(0.1).onTrue(
                Commands.sequence(
                        Commands.print("run intake for close 0 (first note)"),
                        Commands.waitUntil(() -> true).withTimeout(2), // wait until intake has note, or timeout
                        Commands.print("run intake to idle")
                )
        );

        autoTriggers.atTime(1.3).onTrue(
                Commands.sequence(
                        Commands.print("Shoot"),
                        Commands.waitUntil(() -> true),
                        followPath(trajectoryGroup.get(1), timer)
                )
        );

        autoTriggers.atTime(2.2).onTrue(
                Commands.sequence(
                        Commands.print("run intake for close 0 (first note)"),
                        Commands.waitUntil(() -> true).withTimeout(4), // wait until intake has note, or timeout
                        Commands.print("run intake to idle")
                )
        );

        autoTriggers.atTime(5.81).onTrue(
                Commands.sequence(
                        Commands.print("shoot"),
                        Commands.waitUntil(() -> true).withTimeout(1),
                        followPath(trajectoryGroup.get(2), timer)
                )
        );

        autoTriggers.atTime(7).onTrue(
                Commands.sequence(
                        Commands.print("run intake for close 0 (first note)"),
                        Commands.waitUntil(() -> true).withTimeout(4), // wait until intake has note, or timeout
                        Commands.print("run intake to idle")
                )
        );

        autoTriggers.atTime(10.4).onTrue(
                Commands.sequence(
                        Commands.print("shoot"),
                        Commands.waitUntil(() -> true).withTimeout(1)
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
                        Commands.waitUntil(() -> true), // wait until shot goes out
                        followPath(trajectoryGroup.get(0), timer)
                )
        );

        return Commands.run(autoTriggers::poll)
                .until(() -> !DriverStation.isAutonomousEnabled());
    }
}
