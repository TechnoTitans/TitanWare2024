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
    }

    private Command followPath(final ChoreoTrajectory choreoTrajectory, final Timer timer) {
        return Commands.sequence(
                Commands.runOnce(timer::start),
                swerve.followChoreoPathCommand(choreoTrajectory),
                Commands.runOnce(timer::stop)
        );
    }

    public Command sourceBoth() {
        final ChoreoTrajectory sourceBoth = Choreo.getTrajectory("SourceBoth");
        final List<ChoreoTrajectory> sourceBothGroup = Choreo.getTrajectoryGroup("SourceBoth");

        final Timer timer = new Timer();
        final AutoTriggers autoTriggers = new AutoTriggers(sourceBoth, swerve::getPose, timer::get);

        final ChoreoTrajectoryState initialState = sourceBoth.getInitialState();
        autoTriggers.autoEnabled().whileTrue(
                Commands.defer(() -> swerve.resetPoseCommand(
                        IsRedAlliance.getAsBoolean()
                                ? initialState.flipped().getPose()
                                : initialState.getPose()
                        ),
                        Set.of(swerve)
                )
        );

        final ChoreoTrajectory preloadToSpeakerScore0 = sourceBothGroup.get(0);
        autoTriggers.atPlaceAndTime(0).onTrue(
                Commands.sequence(
                    Commands.print("shoot preload"),
                    Commands.waitUntil(() -> true), // wait until shot goes out
                    followPath(preloadToSpeakerScore0, timer)
                )
        );

        autoTriggers.atPlaceAndTime(0.27).onTrue(
                Commands.sequence(
                        Commands.print("run intake for close 0 (first note)"),
                        Commands.waitUntil(() -> true).withTimeout(1), // wait until intake has note, or timeout
                        Commands.print("run intake to idle")
                )
        );

        final ChoreoTrajectory speakerScore0ToSpeakerScore1 = sourceBothGroup.get(1);
        autoTriggers.atPlaceAndTime(1.475).onTrue(
                Commands.sequence(
                        Commands.print("shoot speaker0"),
                        followPath(speakerScore0ToSpeakerScore1, timer)
                )
        );

        autoTriggers.atPlaceAndTime(3.46).onTrue(
                Commands.sequence(
                        Commands.print("run intake for speaker1"),
                        Commands.waitUntil(() -> true).withTimeout(1), // wait until intake has note, or timeout
                        Commands.print("run intake to idle")
                )
        );

        final ChoreoTrajectory speakerScore1ToSpeakerScore2 = sourceBothGroup.get(2);
        autoTriggers.atPlaceAndTime(5.91).onTrue(
                Commands.sequence(
                        Commands.print("shoot speaker1"),
                        followPath(speakerScore1ToSpeakerScore2, timer)
                )
        );

        autoTriggers.atPlaceAndTime(7.00).onTrue(
                Commands.sequence(
                        Commands.print("run intake for speaker2"),
                        Commands.waitUntil(() -> true).withTimeout(1), // wait until intake has note, or timeout
                        Commands.print("run intake to idle")
                )
        );

        autoTriggers.atPlaceAndTime(9.3).onTrue(
                Commands.sequence(
                        Commands.print("shoot speaker2")
                )
        );

        return Commands.run(autoTriggers::poll)
                .until(autoTriggers.autoEnabled().negate());
    }
}
