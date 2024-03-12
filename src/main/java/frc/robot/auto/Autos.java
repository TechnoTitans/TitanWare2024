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
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.superstructure.Superstructure;
import org.littletonrobotics.junction.Logger;

import java.util.List;
import java.util.Optional;
import java.util.Set;
import java.util.concurrent.ThreadLocalRandom;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class Autos {
    private static final double TranslationToleranceMeters = 0.5;
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
                Commands.runOnce(() -> Logger.recordOutput("Auto/FollowingPath", true)),
                swerve.followChoreoPathCommand(choreoTrajectory),
                Commands.runOnce(() -> Logger.recordOutput("Auto/FollowingPath", false)),
                Commands.runOnce(timer::stop)
        );
    }

    private Command shoot() {
        return Commands.deadline(
                intake
                        .runStopCommand()
                        .until(superstructure.atSetpoint)
                        .andThen(intake.feedCommand())
                        .andThen(Commands.waitSeconds(0.2)),
                superstructure.toGoal(Superstructure.Goal.SUBWOOFER)
        );
    }

    private Command resetPose(final ChoreoTrajectoryState initialState) {
        return Commands.defer(() -> swerve.resetPoseCommand(
                        IsRedAlliance.getAsBoolean()
                                ? initialState.flipped().getPose()
                                : initialState.getPose()
                ),
                Set.of()
        );
    }

    public EventLoop doNothing() {
        final EventLoop doNothingEventLoop = new EventLoop();
        new Trigger(doNothingEventLoop, DriverStation::isAutonomousEnabled)
                .whileTrue(
                        Commands.sequence(
                                superstructure.home(),
                                Commands.waitUntil(() -> !DriverStation.isAutonomousEnabled())
                        )
                );

        return doNothingEventLoop;
    }
    
    public EventLoop sourceSpeaker0() {
        final String trajectoryName = "SourceSpeaker0";
        final ChoreoTrajectory trajectory = Choreo.getTrajectory(trajectoryName);
        final List<ChoreoTrajectory> trajectoryGroup = Choreo.getTrajectoryGroup(trajectoryName);

        final Timer timer = new Timer();
        final AutoTriggers autoTriggers = new AutoTriggers(trajectory, swerve::getPose, timer::get);

        autoTriggers.autoEnabled().whileTrue(
                Commands.sequence(
                        resetPose(trajectory.getInitialState()),
                        superstructure.home(),
                        Commands.print("shoot preload"),
                        shoot(),
                        followPath(trajectoryGroup.get(0), timer)
                ).withName("PreloadAndFollow0")
        );

        autoTriggers.atTime(0.1).onTrue(
                Commands.sequence(
                        Commands.print("run intake"),
                        intake.intakeCommand()
                ).withName("Intake0")
        );

        autoTriggers.atTime(1.6).onTrue(
                Commands.sequence(
                        Commands.print("Shoot"),
                        shoot()
                ).withName("Shoot0")
        );

        return autoTriggers.eventLoop;
    }

    public EventLoop ampSpeaker0() {
        final String trajectoryName = "AmpSource0";
        final ChoreoTrajectory trajectory = Choreo.getTrajectory(trajectoryName);
        final List<ChoreoTrajectory> trajectoryGroup = Choreo.getTrajectoryGroup(trajectoryName);

        final Timer timer = new Timer();
        final AutoTriggers autoTriggers = new AutoTriggers(trajectory, swerve::getPose, timer::get);

        // Kraken X60 FOC - 1.163975155279503 Nm @ 60A
        autoTriggers.autoEnabled().whileTrue(
                Commands.sequence(
                        resetPose(trajectory.getInitialState()),
                        superstructure.home().asProxy(),
                        shoot().withName("ShootPreload").asProxy(),
                        followPath(trajectory, timer)
                )
        );

        return autoTriggers.eventLoop;
    }

    public EventLoop sourceSpeaker0Center1() {
        final String trajectoryName = "SourceSpeaker0Center1";
        final ChoreoTrajectory trajectory = Choreo.getTrajectory(trajectoryName);
        final List<ChoreoTrajectory> trajectoryGroup = Choreo.getTrajectoryGroup(trajectoryName);

        final Timer timer = new Timer();
        final AutoTriggers autoTriggers = new AutoTriggers(trajectory, swerve::getPose, timer::get);

        autoTriggers.autoEnabled().whileTrue(
                Commands.sequence(
                        resetPose(trajectory.getInitialState()),
                        superstructure.home(),
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

        return autoTriggers.eventLoop;
    }

    public EventLoop sourceSpeaker0Center1_2() {
        final String trajectoryName = "SourceSpeaker0Center1_2";
        final ChoreoTrajectory trajectory = Choreo.getTrajectory(trajectoryName);
        final List<ChoreoTrajectory> trajectoryGroup = Choreo.getTrajectoryGroup(trajectoryName);

        final Timer timer = new Timer();
        final AutoTriggers autoTriggers = new AutoTriggers(trajectory, swerve::getPose, timer::get);

        autoTriggers.autoEnabled().whileTrue(
                Commands.sequence(
                        resetPose(trajectory.getInitialState()),
                        superstructure.home(),
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

        return autoTriggers.eventLoop;
    }

    public EventLoop walton() {
        final String trajectoryName = "Walton";
        final ChoreoTrajectory trajectory = Choreo.getTrajectory(trajectoryName);
        final List<ChoreoTrajectory> trajectoryGroup = Choreo.getTrajectoryGroup(trajectoryName);

        final Timer timer = new Timer();
        final AutoTriggers autoTriggers = new AutoTriggers(trajectory, swerve::getPose, timer::get);

        autoTriggers.autoEnabled().whileTrue(
                Commands.sequence(
                        resetPose(trajectory.getInitialState()),
                        superstructure.home(),
                        Commands.print("shoot preload"),
                        shoot(),
                        followPath(trajectoryGroup.get(0), timer)
                )
        );

        return autoTriggers.eventLoop;
    }

    public EventLoop speaker0_1_2_path() {
        final String trajectoryName = "Speaker0_1_2";
        final ChoreoTrajectory trajectory = Choreo.getTrajectory(trajectoryName);

        final Timer timer = new Timer();
        final AutoTriggers autoTriggers = new AutoTriggers(trajectory, swerve::getPose, timer::get);

        autoTriggers.autoEnabled().whileTrue(
                Commands.sequence(
                        resetPose(trajectory.getInitialState()),
                        superstructure.home().asProxy(),
                        shoot().withName("ShootPreload").asProxy(),
                        followPath(trajectory, timer)
                ).withName("Follow0_1_2")
        );

        return autoTriggers.eventLoop;
    }

    public EventLoop speaker0_1_2() {
        final String trajectoryName = "Speaker0_1_2";
        final ChoreoTrajectory trajectory = Choreo.getTrajectory(trajectoryName);
        final List<ChoreoTrajectory> trajectoryGroup = Choreo.getTrajectoryGroup(trajectoryName);

        final Timer timer = new Timer();
        final AutoTriggers autoTriggers = new AutoTriggers(trajectory, swerve::getPose, timer::get);

        autoTriggers.autoEnabled().whileTrue(
                Commands.sequence(
                        resetPose(trajectory.getInitialState()),
                        superstructure.home().asProxy(),
                        shoot().withName("ShootPreload").asProxy(),
                        followPath(trajectoryGroup.get(0), timer)
                ).withName("PreloadAndFollow0")
        );

        autoTriggers.atTime(0.3).onTrue(
                Commands.sequence(
                        intake.intakeCommand()
                ).withName("Intake0")
        );

        autoTriggers.atTime(1.785).onTrue(
                Commands.sequence(
                        shoot().withName("Shoot0").asProxy(),
                        followPath(trajectoryGroup.get(1), timer)
                ).withName("Shoot0AndFollow1")
        );

        autoTriggers.atTime(2).onTrue(
                Commands.sequence(
                        intake.intakeCommand()
                ).withName("Intake1")
        );

        autoTriggers.atTime(3.165).onTrue(
                Commands.sequence(
                        shoot().withName("Shoot1").asProxy(),
                        followPath(trajectoryGroup.get(2), timer)
                ).withName("Shoot1AndFollow2")
        );

        autoTriggers.atTime(3.5).onTrue(
                Commands.sequence(
                        intake.intakeCommand()
                ).withName("Intake2")
        );

        autoTriggers.atTime(5.28).onTrue(
                Commands.sequence(
                        shoot().withName("Shoot2").asProxy()
                ).withName("Shoot2")
        );

        return autoTriggers.eventLoop;
    }

    public EventLoop ampSpeaker2Center3() {
        final String trajectoryName = "AmpSpeaker2Center3";
        final ChoreoTrajectory trajectory = Choreo.getTrajectory(trajectoryName);
        final List<ChoreoTrajectory> trajectoryGroup = Choreo.getTrajectoryGroup(trajectoryName);

        final Timer timer = new Timer();
        final AutoTriggers autoTriggers = new AutoTriggers(trajectory, swerve::getPose, timer::get);

        autoTriggers.autoEnabled().whileTrue(
                Commands.sequence(
                        resetPose(trajectory.getInitialState()),
                        superstructure.home(),
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

        return autoTriggers.eventLoop;
    }

    public EventLoop shootAndMobility() {
        final String trajectoryName = "ShootAndMobility";
        final ChoreoTrajectory trajectory = Choreo.getTrajectory(trajectoryName);
        final List<ChoreoTrajectory> trajectoryGroup = Choreo.getTrajectoryGroup(trajectoryName);

        final Timer timer = new Timer();
        final AutoTriggers autoTriggers = new AutoTriggers(trajectory, swerve::getPose, timer::get);

        autoTriggers.autoEnabled()
                .whileTrue(Commands.sequence(
                        resetPose(trajectory.getInitialState()),
                        superstructure.home(),
                        Commands.print("shoot preload"),
                        shoot(),
                        Commands.print("start following! time: " + trajectoryGroup.get(0).getTotalTime()),
                        followPath(trajectoryGroup.get(0), timer),
                        Commands.print("done following!")
                ));

        return autoTriggers.eventLoop;
    }
}
