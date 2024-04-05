package frc.robot.auto;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
import frc.robot.ShootCommands;
import frc.robot.state.NoteState;
import frc.robot.subsystems.drive.Swerve;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.superstructure.Superstructure;
import org.littletonrobotics.junction.Logger;

import java.util.List;
import java.util.Set;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

@SuppressWarnings("DuplicatedCode")
public class Autos {
    public static final String LogKey = "Auto";

    private static final double TranslationToleranceMeters = 0.5;
    private static final double TimeToleranceSeconds = 0.1;

    private final Swerve swerve;
    private final Intake intake;
    private final Superstructure superstructure;

    private final NoteState noteState;

    private final ShootCommands shootCommands;

    public Autos(
            final Swerve swerve,
            final Intake intake,
            final Superstructure superstructure,
            final NoteState noteState,
            final ShootCommands shootCommands
    ) {
        this.swerve = swerve;
        this.intake = intake;
        this.superstructure = superstructure;
        this.noteState = noteState;

        this.shootCommands = shootCommands;
    }

    private static class AutoTriggers {
        private final ChoreoTrajectory trajectory;
        private final List<ChoreoTrajectory> trajectories;
        private final Supplier<Pose2d> poseSupplier;
        private final DoubleSupplier timeSupplier;
        private final EventLoop eventLoop;

        public AutoTriggers(
                final ChoreoTrajectory trajectory,
                final List<ChoreoTrajectory> trajectories,
                final Supplier<Pose2d> poseSupplier,
                final DoubleSupplier timeSupplier
        ) {
            this.trajectory = trajectory;
            this.trajectories = trajectories;
            this.poseSupplier = poseSupplier;
            this.timeSupplier = timeSupplier;
            this.eventLoop = new EventLoop();
        }

        public AutoTriggers(
                final String trajectoryName,
                final Supplier<Pose2d> poseSupplier,
                final DoubleSupplier timeSupplier
        ) {
            this(
                    Choreo.getTrajectory(trajectoryName),
                    Choreo.getTrajectoryGroup(trajectoryName),
                    poseSupplier,
                    timeSupplier
            );
        }

        public Trigger autoEnabled() {
            return new Trigger(eventLoop, DriverStation::isAutonomousEnabled);
        }

        //TODO: Fix this it doesn't work
        @SuppressWarnings("unused")
        public Trigger atPlaceAndTime(final double timeSeconds) {
            final Translation2d place = trajectory
                    .sample(timeSeconds, Robot.IsRedAlliance.getAsBoolean())
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
        @SuppressWarnings("unused")
        public Trigger atPlace(final double timeSeconds) {
            final Translation2d place = trajectory
                    .sample(timeSeconds, Robot.IsRedAlliance.getAsBoolean())
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
        return Commands.runOnce(timer::start)
                .andThen(swerve.followChoreoPathCommand(choreoTrajectory, Robot.IsRedAlliance))
                .finallyDo(timer::stop);
    }

    private Command resetPose(final ChoreoTrajectory trajectory) {
        return Commands.defer(() -> swerve.resetPoseCommand(
                        Robot.IsRedAlliance.getAsBoolean()
                                ? trajectory.getFlippedInitialPose()
                                : trajectory.getInitialPose()
                ),
                Set.of()
        );
    }

    private Command preloadSubwooferAndFollow0(
            final List<ChoreoTrajectory> trajectories,
            final Timer timer
    ) {
        final ChoreoTrajectory trajectory0 = trajectories.get(0);
        return Commands.sequence(
                noteState.setState(NoteState.State.STORED),
                resetPose(trajectory0),
                shootCommands.shootSubwoofer().withName("ShootPreload").asProxy(),
                followPath(trajectory0, timer)
        ).withName("PreloadAndFollow0");
    }

    private Command followIntakeAndInstantShoot(
            final ChoreoTrajectory trajectory,
            final Timer timer,
            final double intakeTimeSeconds
    ) {
        return Commands.parallel(
                followPath(trajectory, timer),
                Commands.sequence(
                        Commands.waitUntil(() -> MathUtil.isNear(intakeTimeSeconds, timer.get(), TimeToleranceSeconds)),
                        Commands.runOnce(() -> swerve.setPathHeadingOverride(
                                ShootCommands.angleToSpeakerSupplier(swerve::getPose)
                        )),
                        Commands.deadline(
                                intake.intakeAndFeedCommand(),
                                superstructure.runState(ShootCommands.shotParametersSupplier(swerve::getPose))
                        )
                ).finallyDo(swerve::clearPathHeadingOverride)
        );
    }

    public EventLoop doNothing() {
        final EventLoop doNothingEventLoop = new EventLoop();
        new Trigger(doNothingEventLoop, DriverStation::isAutonomousEnabled)
                .whileTrue(Commands.waitUntil(() -> !DriverStation.isAutonomousEnabled()));

        return doNothingEventLoop;
    }
    
    public EventLoop sourceSpeaker0() {
        final Timer timer = new Timer();
        final AutoTriggers autoTriggers = new AutoTriggers("SourceSpeaker0", swerve::getPose, timer::get);

        autoTriggers.autoEnabled().whileTrue(preloadSubwooferAndFollow0(autoTriggers.trajectories, timer));

        autoTriggers.atTime(0.3).onTrue(
                Commands.sequence(
                        intake.intakeCommand()
                ).withName("Intake0")
        );

        autoTriggers.atTime(2.45).onTrue(
                Commands.sequence(
                        shootCommands.deferredStopAimAndShoot().asProxy()
                ).withName("Shoot0")
        );

        return autoTriggers.eventLoop;
    }

    public EventLoop ampSpeaker2() {
        final Timer timer = new Timer();
        final AutoTriggers autoTriggers = new AutoTriggers("AmpSpeaker2", swerve::getPose, timer::get);

        autoTriggers.autoEnabled().whileTrue(preloadSubwooferAndFollow0(autoTriggers.trajectories, timer));

        autoTriggers.atTime(0.3).onTrue(
                Commands.sequence(
                        intake.intakeCommand()
                ).withName("Intake0")
        );

        autoTriggers.atTime(2.31).onTrue(
                Commands.sequence(
                        shootCommands.deferredStopAimAndShoot().asProxy()
                ).withName("Shoot0")
        );

        return autoTriggers.eventLoop;
    }

    public EventLoop sourceSpeaker0Center1() {
        final Timer timer = new Timer();
        final AutoTriggers autoTriggers = new AutoTriggers("SourceSpeaker0Center1", swerve::getPose, timer::get);

        autoTriggers.autoEnabled().whileTrue(preloadSubwooferAndFollow0(autoTriggers.trajectories, timer));

        autoTriggers.atTime(0.3).onTrue(
                Commands.sequence(
                        intake.intakeCommand()
                ).withName("Intake0")
        );

        autoTriggers.atTime(2.34).onTrue(
                Commands.sequence(
                        shootCommands.deferredStopAimAndShoot().asProxy(),
                        followPath(autoTriggers.trajectories.get(1), timer)
                ).withName("Shoot0Follow1")
        );

        autoTriggers.atTime(3.8).onTrue(
                Commands.sequence(
                        intake.intakeCommand()
                ).withName("Intake1")
        );

        autoTriggers.atTime(8.4).onTrue(
                Commands.sequence(
                        shootCommands.deferredStopAimAndShoot().asProxy()
                ).withName("Shoot2")
        );

        return autoTriggers.eventLoop;
    }

    public EventLoop sourceSpeaker0Center1_2() {
        final Timer timer = new Timer();
        final AutoTriggers autoTriggers = new AutoTriggers("SourceSpeaker0Center1_2", swerve::getPose, timer::get);

        autoTriggers.autoEnabled().whileTrue(preloadSubwooferAndFollow0(autoTriggers.trajectories, timer));

        autoTriggers.atTime(0.3).onTrue(
                Commands.sequence(
                        intake.intakeCommand()
                ).withName("Intake0")
        );

        autoTriggers.atTime(2.06).onTrue(
                Commands.sequence(
                        shootCommands.deferredStopAimAndShoot().asProxy(),
                        followPath(autoTriggers.trajectories.get(1), timer)
                ).withName("Shoot0Follow1")
        );

        autoTriggers.atTime(3.6).onTrue(
                Commands.sequence(
                        intake.intakeCommand()
                ).withName("Intake1")
        );

        autoTriggers.atTime(7.6).onTrue(
                Commands.sequence(
                        shootCommands.deferredStopAimAndShoot().asProxy(),
                        followPath(autoTriggers.trajectories.get(2), timer)
                ).withName("Shoot1Follow2")
        );

        autoTriggers.atTime(9.6).onTrue(
                Commands.sequence(
                        intake.intakeCommand()
                ).withName("Intake2")
        );

        autoTriggers.atTime(14.24).onTrue(
                Commands.sequence(
                        shootCommands.deferredStopAimAndShoot().asProxy()
                ).withName("Shoot2")
        );

        return autoTriggers.eventLoop;
    }

    public EventLoop walton() {
        final Timer timer = new Timer();
        final AutoTriggers autoTriggers = new AutoTriggers("Walton", swerve::getPose, timer::get);

        autoTriggers.autoEnabled().whileTrue(preloadSubwooferAndFollow0(autoTriggers.trajectories, timer));

        return autoTriggers.eventLoop;
    }

    public EventLoop ampSpeaker2Center2_3() {
        final Timer timer = new Timer();
        final AutoTriggers autoTriggers = new AutoTriggers("AmpSpeaker2Center2_3", swerve::getPose, timer::get);

        autoTriggers.autoEnabled().whileTrue(preloadSubwooferAndFollow0(autoTriggers.trajectories, timer));

        autoTriggers.atTime(0.4).onTrue(
                Commands.sequence(
                        intake.intakeCommand(),
                        shootCommands.readySuperstructureForShot()
                )
        );

        autoTriggers.atTime(1.28).onTrue(
                Commands.sequence(
                        shootCommands.deferredStopAimAndShoot().asProxy(),
                        followPath(autoTriggers.trajectories.get(1), timer)
                )
        );

        autoTriggers.atTime(3).onTrue(
                Commands.sequence(
                        intake.intakeCommand(),
                        shootCommands.readySuperstructureForShot()
                )
        );

        autoTriggers.atTime(6.19).onTrue(
                Commands.sequence(
                        shootCommands.deferredStopAimAndShoot().asProxy()
                )
        );

        autoTriggers.atTime(7).onTrue(
                Commands.sequence(
                        intake.intakeCommand(),
                        shootCommands.readySuperstructureForShot()
                )
        );

        autoTriggers.atTime(9.62).onTrue(
                Commands.sequence(
                        shootCommands.deferredStopAimAndShoot().asProxy()
                )
        );

        return autoTriggers.eventLoop;
    }

    public EventLoop speaker2_1_0() {
        final Timer timer = new Timer();
        final AutoTriggers autoTriggers = new AutoTriggers("Speaker2_1_0", swerve::getPose, timer::get);

        autoTriggers.autoEnabled().whileTrue(preloadSubwooferAndFollow0(autoTriggers.trajectories, timer));

        autoTriggers.atTime(0.4).onTrue(
                Commands.parallel(
                        intake.intakeCommand(),
                        shootCommands.readySuperstructureForShot()
                ).withName("Intake0")
        );

        autoTriggers.atTime(1.25).onTrue(
                Commands.sequence(
//                        Commands.waitUntil(noteState.isStored)
//                                .withTimeout(2),
                        Commands.waitSeconds(0.5),
                        shootCommands.deferredStopAimAndShoot().withName("Shoot0").asProxy(),
                        followPath(autoTriggers.trajectories.get(1), timer).asProxy()
                ).withName("Shoot0AndFollow1")
        );

        autoTriggers.atTime(2).onTrue(
                Commands.parallel(
                        intake.intakeCommand(),
                        shootCommands.readySuperstructureForShot()
                ).withName("Intake1")
        );

        autoTriggers.atTime(3.87).onTrue(
                Commands.sequence(
//                        Commands.waitUntil(noteState.isStored)
//                                .withTimeout(2),
                        Commands.waitSeconds(0.5),
                        shootCommands.deferredStopAimAndShoot().withName("Shoot1").asProxy(),
                        followPath(autoTriggers.trajectories.get(2), timer).asProxy()
                ).withName("Shoot1AndFollow2")
        );

        autoTriggers.atTime(4.3).onTrue(
                Commands.parallel(
                        intake.intakeCommand(),
                        shootCommands.readySuperstructureForShot()
                ).withName("Intake2")
        );

        autoTriggers.atTime(6.02).onTrue(
                Commands.sequence(
//                        Commands.waitUntil(noteState.isStored)
//                                .withTimeout(2),
                        Commands.waitSeconds(0.5),
                        shootCommands.deferredStopAimAndShoot().withName("Shoot2").asProxy()
                ).withName("Shoot2")
        );

        return autoTriggers.eventLoop;
    }

    public EventLoop sourceMobility() {
        final Timer timer = new Timer();
        final AutoTriggers autoTriggers = new AutoTriggers("SourceMobility", swerve::getPose, timer::get);

        autoTriggers.autoEnabled()
                .whileTrue(preloadSubwooferAndFollow0(autoTriggers.trajectories, timer));

        return autoTriggers.eventLoop;
    }

    public EventLoop sourceCenter1_0() {
        final Timer timer = new Timer();
        final AutoTriggers autoTriggers = new AutoTriggers("SourceCenter1_0", swerve::getPose, timer::get);

        autoTriggers.autoEnabled().whileTrue(preloadSubwooferAndFollow0(autoTriggers.trajectories, timer));

        autoTriggers.atTime(1.7).onTrue(
                Commands.parallel(
                        intake.intakeCommand(),
                        shootCommands.readySuperstructureForShot()
                ).withName("Intake0")
        );

        autoTriggers.atTime(5.18).onTrue(
                Commands.sequence(
                        shootCommands.deferredStopAimAndShoot().withName("Shoot0").asProxy(),
                        followPath(autoTriggers.trajectories.get(1), timer).asProxy()
                ).withName("Shoot0AndFollow1")
        );

        autoTriggers.atTime(6).onTrue(
                Commands.parallel(
                        intake.intakeCommand(),
                        shootCommands.readySuperstructureForShot()
                ).withName("Intake1")
        );

        autoTriggers.atTime(9.51).onTrue(
                Commands.sequence(
                        shootCommands.deferredStopAimAndShoot().withName("Shoot1").asProxy()
                ).withName("Shoot1")
        );

        return autoTriggers.eventLoop;
    }

    public EventLoop ampCenter3_2() {
        final Timer timer = new Timer();
        final AutoTriggers autoTriggers = new AutoTriggers("AmpCenter3_2", swerve::getPose, timer::get);

        autoTriggers.autoEnabled().whileTrue(preloadSubwooferAndFollow0(autoTriggers.trajectories, timer));

        autoTriggers.atTime(2).onTrue(
                Commands.parallel(
                        intake.intakeCommand(),
                        shootCommands.readySuperstructureForShot()
                ).withName("Intake0")
        );

        autoTriggers.atTime(4.98).onTrue(
                Commands.sequence(
                        shootCommands.deferredStopAimAndShoot().withName("Shoot0").asProxy(),
                        followPath(autoTriggers.trajectories.get(1), timer).asProxy()
                ).withName("Shoot0AndFollow1")
        );

        autoTriggers.atTime(6).onTrue(
                Commands.parallel(
                        intake.intakeCommand(),
                        shootCommands.readySuperstructureForShot()
                ).withName("Intake1")
        );

        autoTriggers.atTime(9.29).onTrue(
                Commands.sequence(
                        shootCommands.deferredStopAimAndShoot().withName("Shoot1").asProxy()
                ).withName("Shoot1AndFollow2")
        );

        return autoTriggers.eventLoop;
    }

    public EventLoop ampCenter4_3() {
        final Timer timer = new Timer();
        final AutoTriggers autoTriggers = new AutoTriggers("AmpCenter4_3", swerve::getPose, timer::get);

        autoTriggers.autoEnabled().whileTrue(preloadSubwooferAndFollow0(autoTriggers.trajectories, timer));

        autoTriggers.atTime(1.8).onTrue(
                Commands.parallel(
                        intake.intakeCommand(),
                        shootCommands.readySuperstructureForShot()
                ).withName("Intake0")
        );

        autoTriggers.atTime(4.54).onTrue(
                Commands.sequence(
                        shootCommands.deferredStopAimAndShoot()
                                .onlyIf(noteState.hasNote)
                                .withName("Shoot0")
                                .asProxy(),
                        followPath(autoTriggers.trajectories.get(1), timer).asProxy()
                ).withName("Shoot0AndFollow1")
        );

        autoTriggers.atTime(5.2).onTrue(
                Commands.parallel(
                        intake.intakeCommand(),
                        shootCommands.readySuperstructureForShot()
                ).withName("Intake1")
        );

        autoTriggers.atTime(8.02).onTrue(
                Commands.sequence(
                        shootCommands.deferredStopAimAndShoot()
                                .onlyIf(noteState.hasNote)
                                .withName("Shoot1")
                                .asProxy()
                ).withName("Shoot1AndFollow2")
        );

        return autoTriggers.eventLoop;
    }

    public EventLoop speaker0_1_2Center4_3() {
        final String trajectoryName = "Speaker0_1_2Center4_3";
        final Timer timer = new Timer();
        final AutoTriggers autoTriggers = new AutoTriggers(trajectoryName, swerve::getPose, timer::get);

        autoTriggers.autoEnabled().whileTrue(
            Commands.sequence(
                    noteState.setState(NoteState.State.STORED),
                    resetPose(autoTriggers.trajectory),
                    shootCommands.shootSubwoofer().withName("ShootPreload").asProxy(),
                    followIntakeAndInstantShoot(
                            autoTriggers.trajectories.get(0),
                            timer,
                            0.4
                    ).withName("Follow0AndIntakeInstantShoot0").asProxy()
            ).withName("PreloadFollow0AndShoot0")
        );

        autoTriggers.atTime(1.13).onTrue(
                Commands.parallel(
                        followIntakeAndInstantShoot(
                                autoTriggers.trajectories.get(1),
                                timer,
                                1.5
                        ).withName("Follow1AndIntakeInstantShoot1")
                ).withName("Follow1Intake1AndShoot1")
        );

        autoTriggers.atTime(2.61).onTrue(
                Commands.parallel(
                        followIntakeAndInstantShoot(
                                autoTriggers.trajectories.get(2),
                                timer,
                                3
                        ).withName("Follow2AndIntakeInstantShoot2").asProxy()
                ).withName("Follow2Intake2AndShoot2")
        );

        autoTriggers.atTime(4.04).onTrue(
                Commands.sequence(
                        followPath(autoTriggers.trajectories.get(3), timer).asProxy()
                ).withName("Follow3")
        );

        autoTriggers.atTime(5).onTrue(
                Commands.parallel(
                        intake.intakeCommand()
                ).withName("Intake3")
        );

        autoTriggers.atTime(7.82).onTrue(
                Commands.sequence(
                        shootCommands.deferredStopAimAndShoot().withName("Shoot3").asProxy(),
                        followPath(autoTriggers.trajectories.get(4), timer).asProxy()
                ).withName("Shoot3AndFollow4")
        );

        autoTriggers.atTime(8.5).onTrue(
                Commands.parallel(
                        intake.intakeCommand()
                ).withName("Intake4")
        );

        autoTriggers.atTime(11.2).onTrue(
                Commands.sequence(
                        shootCommands.deferredStopAimAndShoot().withName("Shoot3").asProxy()
                ).withName("Shoot4")
        );

        return autoTriggers.eventLoop;
    }

    public EventLoop ampSpeaker2_1Center2_3_4() {
        final String trajectoryName = "AmpSpeaker2_1Center2_3_4";
        final Timer timer = new Timer();
        final AutoTriggers autoTriggers = new AutoTriggers(trajectoryName, swerve::getPose, timer::get);

        autoTriggers.autoEnabled().whileTrue(
                Commands.sequence(
                        noteState.setState(NoteState.State.STORED),
                        resetPose(autoTriggers.trajectory),
                        shootCommands.shootSubwoofer().withName("ShootPreload").asProxy(),
                        followIntakeAndInstantShoot(
                                autoTriggers.trajectories.get(0),
                                timer,
                                0.4
                        ).withName("Follow0AndIntakeInstantShoot0")
                ).withName("PreloadFollow0AndShoot0")
        );

        autoTriggers.atTime(1.21).onTrue(
                Commands.parallel(
                        followIntakeAndInstantShoot(
                                autoTriggers.trajectories.get(1),
                                timer,
                                1.6
                        ).withName("Follow1AndIntakeInstantShoot1")
                ).withName("Follow1Intake1AndShoot1")
        );

        autoTriggers.atTime(2.7).onTrue(
                Commands.sequence(
                        followPath(autoTriggers.trajectories.get(2), timer).asProxy()
                ).withName("Follow2")
        );

        autoTriggers.atTime(3.8).onTrue(
                Commands.parallel(
                        intake.intakeCommand()
                ).withName("Intake2")
        );

        autoTriggers.atTime(6.42).onTrue(
                Commands.sequence(
                        shootCommands.deferredStopAimAndShoot().withName("Shoot2").asProxy(),
                        followPath(autoTriggers.trajectories.get(3), timer).asProxy()
                ).withName("Shoot2AndFollow3")
        );

        autoTriggers.atTime(6.9).onTrue(
                Commands.parallel(
                        intake.intakeCommand()
                ).withName("Intake3")
        );

        autoTriggers.atTime(9.31).onTrue(
                Commands.sequence(
                        shootCommands.deferredStopAimAndShoot().withName("Shoot3").asProxy(),
                        followPath(autoTriggers.trajectories.get(4), timer).asProxy()
                ).withName("Shoot3AndFollow4")
        );

        autoTriggers.atTime(9.8).onTrue(
                Commands.parallel(
                        intake.intakeCommand()
                ).withName("Intake4")
        );

        autoTriggers.atTime(12.2).onTrue(
                Commands.sequence(
                        shootCommands.deferredStopAimAndShoot().withName("Shoot4").asProxy()
                ).withName("Shoot4")
        );

        return autoTriggers.eventLoop;
    }

    public EventLoop speaker0_1_2Center4_3_2() {
        final String trajectoryName = "Speaker0_1_2Center4_3_2";
        final Timer timer = new Timer();
        final AutoTriggers autoTriggers = new AutoTriggers(trajectoryName, swerve::getPose, timer::get);

        autoTriggers.autoEnabled().whileTrue(
                Commands.sequence(
                        noteState.setState(NoteState.State.STORED),
                        resetPose(autoTriggers.trajectory),
                        shootCommands.shootSubwoofer().withName("ShootPreload").asProxy(),
                        followIntakeAndInstantShoot(
                                autoTriggers.trajectories.get(0),
                                timer,
                                0.4
                        ).withName("Follow0AndIntakeInstantShoot0")
                ).withName("PreloadFollow0AndShoot0")
        );

        autoTriggers.atTime(1.13).onTrue(
                Commands.parallel(
                        followIntakeAndInstantShoot(
                                autoTriggers.trajectories.get(1),
                                timer,
                                1.5
                        ).withName("Follow1AndIntakeInstantShoot1")
                ).withName("Follow1Intake1AndShoot1")
        );

        autoTriggers.atTime(2.61).onTrue(
                Commands.parallel(
                        followIntakeAndInstantShoot(
                                autoTriggers.trajectories.get(2),
                                timer,
                                3
                        ).withName("Follow2AndIntakeInstantShoot2")
                ).withName("Follow2Intake2AndShoot2")
        );

        autoTriggers.atTime(4.04).onTrue(
                Commands.sequence(
                        followPath(autoTriggers.trajectories.get(3), timer).asProxy()
                ).withName("Follow3")
        );

        autoTriggers.atTime(5).onTrue(
                Commands.parallel(
                        intake.intakeCommand()
                ).withName("Intake3")
        );

        autoTriggers.atTime(7.55).onTrue(
                Commands.sequence(
                        shootCommands.deferredStopAimAndShoot().withName("Shoot3").asProxy(),
                        followPath(autoTriggers.trajectories.get(4), timer).asProxy()
                ).withName("Shoot3AndFollow4")
        );

        autoTriggers.atTime(8.2).onTrue(
                Commands.parallel(
                        intake.intakeCommand()
                ).withName("Intake4")
        );

        autoTriggers.atTime(10.4).onTrue(
                Commands.sequence(
                        shootCommands.deferredStopAimAndShoot().withName("Shoot3").asProxy(),
                        followPath(autoTriggers.trajectories.get(5), timer).asProxy()
                ).withName("Shoot4")
        );

        autoTriggers.atTime(11).onTrue(
                Commands.parallel(
                        intake.intakeCommand()
                ).withName("Intake4")
        );

        autoTriggers.atTime(13.65).onTrue(
                Commands.sequence(
                        shootCommands.deferredStopAimAndShoot().withName("Shoot3").asProxy()
                ).withName("Shoot4")
        );

        return autoTriggers.eventLoop;
    }

    public EventLoop sourceCenter0_1_2() {
        final String trajectoryName = "SourceCenter0_1_2";
        final Timer timer = new Timer();
        final AutoTriggers autoTriggers = new AutoTriggers(trajectoryName, swerve::getPose, timer::get);

        autoTriggers.autoEnabled().whileTrue(preloadSubwooferAndFollow0(autoTriggers.trajectories, timer));

        autoTriggers.atTime(1.6).onTrue(
                Commands.parallel(
                        intake.intakeCommand()
                ).withName("Intake0")
        );

        autoTriggers.atTime(5.04).onTrue(
                Commands.sequence(
                        shootCommands.deferredStopAimAndShoot()
                                .onlyIf(noteState.hasNote)
                                .withName("Shoot0")
                                .asProxy(),
                        followPath(autoTriggers.trajectories.get(1), timer).asProxy()
                ).withName("Shoot0AndFollow1")
        );

        autoTriggers.atTime(6).onTrue(
                Commands.parallel(
                        intake.intakeCommand()
                ).withName("Intake1")
        );

        autoTriggers.atTime(9.24).onTrue(
                Commands.sequence(
                        shootCommands.deferredStopAimAndShoot()
                                .onlyIf(noteState.hasNote)
                                .withName("Shoot1")
                                .asProxy(),
                        followPath(autoTriggers.trajectories.get(2), timer).asProxy()
                ).withName("Shoot1AndFollow2")
        );

        autoTriggers.atTime(10).onTrue(
                Commands.parallel(
                        intake.intakeCommand()
                ).withName("Intake2")
        );

        autoTriggers.atTime(13.24).onTrue(
                Commands.sequence(
                        shootCommands.deferredStopAimAndShoot()
                                .onlyIf(noteState.hasNote)
                                .withName("Shoot2")
                                .asProxy()
                ).withName("Shoot2")
        );

        return autoTriggers.eventLoop;
    }
}
