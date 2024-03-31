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
        public Trigger atPlace(final double timeSeconds) {
            final Translation2d place = choreoTrajectory
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



    private Command shoot() {
        return Commands.deadline(
                intake
                        .runStopCommand()
                        .until(superstructure.atSetpoint)
                        .withTimeout(4)
                        .andThen(intake.feedCommand())
                        .andThen(Commands.waitSeconds(0.2)),
//                superstructure.runState(() -> ShotParameters.get(
//                        swerve.getPose()
//                                .minus(FieldConstants.getSpeakerPose())
//                                .getTranslation()
//                                .getNorm()
//                ))
                superstructure.toGoal(Superstructure.Goal.SUBWOOFER)
        );
    }

    private Command resetPose(final ChoreoTrajectoryState initialState) {
        return Commands.defer(() -> swerve.resetPoseCommand(
                        Robot.IsRedAlliance.getAsBoolean()
                                ? initialState.flipped().getPose()
                                : initialState.getPose()
                ),
                Set.of()
        );
    }

    public EventLoop doNothing() {
        final EventLoop doNothingEventLoop = new EventLoop();
        new Trigger(doNothingEventLoop, DriverStation::isAutonomousEnabled)
                .whileTrue(Commands.waitUntil(() -> !DriverStation.isAutonomousEnabled()));

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
                        shoot().asProxy(),
                        followPath(trajectoryGroup.get(0), timer)
                ).withName("PreloadAndFollow0")
        );

        autoTriggers.atTime(0.3).onTrue(
                Commands.sequence(
                        intake.intakeCommand()
                ).withName("Intake0")
        );

        autoTriggers.atTime(1.84).onTrue(
                Commands.sequence(
                        Commands.print("Shoot"),
                        shootCommands.stopAimAndShoot().asProxy()
                ).withName("Shoot0")
        );

        return autoTriggers.eventLoop;
    }

    public EventLoop ampSpeaker2() {
        final String trajectoryName = "AmpSpeaker2";
        final ChoreoTrajectory trajectory = Choreo.getTrajectory(trajectoryName);
        final List<ChoreoTrajectory> trajectoryGroup = Choreo.getTrajectoryGroup(trajectoryName);

        final Timer timer = new Timer();
        final AutoTriggers autoTriggers = new AutoTriggers(trajectory, swerve::getPose, timer::get);

        autoTriggers.autoEnabled().whileTrue(
                Commands.sequence(
                        resetPose(trajectory.getInitialState()),
                        shoot().withName("ShootPreload").asProxy(),
                        followPath(trajectoryGroup.get(0), timer)
                ).withName("PreloadAndFollow0")
        );

        autoTriggers.atTime(0.3).onTrue(
                Commands.sequence(
                        intake.intakeCommand()
                ).withName("Intake0")
        );

        autoTriggers.atTime(2.31).onTrue(
                Commands.sequence(
                        shootCommands.stopAimAndShoot().withName("LineupAndShoot0").asProxy()
                ).withName("Shoot0")
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
                        Commands.print("shoot preload"),
                        shoot(),
                        followPath(trajectoryGroup.get(0), timer)
                )
        );

        return autoTriggers.eventLoop;
    }

    public EventLoop ampSpeaker2Center2_3() {
        final String trajectoryName = "AmpSpeaker2Center2_3";
        final ChoreoTrajectory trajectory = Choreo.getTrajectory(trajectoryName);
        final List<ChoreoTrajectory> trajectoryGroup = Choreo.getTrajectoryGroup(trajectoryName);

        final Timer timer = new Timer();
        final AutoTriggers autoTriggers = new AutoTriggers(trajectory, swerve::getPose, timer::get);

        autoTriggers.autoEnabled().whileTrue(
                Commands.sequence(
                        resetPose(trajectory.getInitialState()),
                        shoot().withName("ShootPreload").asProxy(),
                        followPath(trajectoryGroup.get(0), timer)
                ).withName("ShootPreloadAndFollow0")

        );

        autoTriggers.atTime(0.4).onTrue(
                Commands.sequence(
                        intake.intakeCommand(),
                        shootCommands.readySuperstructureForShot()
                )
        );

        autoTriggers.atTime(1.28).onTrue(
                Commands.sequence(
                        shootCommands.deferredStopAimAndShoot().asProxy(),
                        followPath(trajectoryGroup.get(1), timer)
                )
        );

        autoTriggers.atTime(2.5).onTrue(
                Commands.sequence(
                        intake.intakeCommand(),
                        shootCommands.readySuperstructureForShot()
                )
        );

        autoTriggers.atTime(5.82).onTrue(
                Commands.sequence(
                        shootCommands.deferredStopAimAndShoot().asProxy()
                )
        );

        autoTriggers.atTime(6.5).onTrue(
                Commands.sequence(
                        intake.intakeCommand(),
                        shootCommands.readySuperstructureForShot()
                )
        );

        autoTriggers.atTime(9.34).onTrue(
                Commands.sequence(
                        shootCommands.deferredStopAimAndShoot().asProxy()
                )
        );

        return autoTriggers.eventLoop;
    }

    public EventLoop speaker2_1_0() {
        final String trajectoryName = "Speaker2_1_0";
        final ChoreoTrajectory trajectory = Choreo.getTrajectory(trajectoryName);
        final List<ChoreoTrajectory> trajectoryGroup = Choreo.getTrajectoryGroup(trajectoryName);

        final Timer timer = new Timer();
        final AutoTriggers autoTriggers = new AutoTriggers(trajectory, swerve::getPose, timer::get);

        autoTriggers.autoEnabled().whileTrue(
                Commands.sequence(
                        resetPose(trajectory.getInitialState()),
                        shoot().withName("ShootPreload").asProxy(),
                        followPath(trajectoryGroup.get(0), timer)
                ).withName("PreloadAndFollow0")
        );

        autoTriggers.autoEnabled().whileTrue(
                Commands.run(() -> Logger.recordOutput("Auto/Timer", timer.get()))
        );

        autoTriggers.atTime(0.4).onTrue(
                Commands.parallel(
                        intake.intakeCommand(),
                        shootCommands.readySuperstructureForShot()
                ).withName("Intake0")
        );

        autoTriggers.atTime(1.25).onTrue(
                Commands.sequence(
//                        Commands.waitUntil(noteState.isStored).withTimeout(2),
                        Commands.waitSeconds(0.5),
                        shootCommands.deferredStopAimAndShoot().withName("Shoot0").asProxy(),
                        followPath(trajectoryGroup.get(1), timer).asProxy()
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
//                        Commands.waitUntil(noteState.isStored).withTimeout(2),
                        Commands.waitSeconds(0.5),
                        shootCommands.deferredStopAimAndShoot().withName("Shoot1").asProxy(),
                        followPath(trajectoryGroup.get(2), timer).asProxy()
                ).withName("Shoot1AndFollow2")
        );

        autoTriggers.atTime(4.3).onTrue(
                Commands.parallel(
                        intake.intakeCommand(),
                        shootCommands.readySuperstructureForShot()
                ).withName("Intake2")
        );

        autoTriggers.atTime(6).onTrue(
                Commands.sequence(
//                        Commands.waitUntil(noteState.isStored).withTimeout(2),
                        shootCommands.deferredStopAimAndShoot().withName("Shoot2").asProxy()
                ).withName("Shoot2")
        );

        return autoTriggers.eventLoop;
    }

    public EventLoop sourceMobility() {
        final String trajectoryName = "SourceMobility";
        final ChoreoTrajectory trajectory = Choreo.getTrajectory(trajectoryName);
        final List<ChoreoTrajectory> trajectoryGroup = Choreo.getTrajectoryGroup(trajectoryName);

        final Timer timer = new Timer();
        final AutoTriggers autoTriggers = new AutoTriggers(trajectory, swerve::getPose, timer::get);

        autoTriggers.autoEnabled()
                .whileTrue(
                        Commands.sequence(
                                resetPose(trajectory.getInitialState()),
                                shoot().withName("ShootPreload").asProxy(),
                                followPath(trajectoryGroup.get(0), timer)
                        ).withName("ShootPreloadAndFollow0")
                );

        return autoTriggers.eventLoop;
    }

    public EventLoop sourceCenter1_0() {
        final String trajectoryName = "SourceCenter1_0";
        final ChoreoTrajectory trajectory = Choreo.getTrajectory(trajectoryName);
        final List<ChoreoTrajectory> trajectoryGroup = Choreo.getTrajectoryGroup(trajectoryName);

        final Timer timer = new Timer();
        final AutoTriggers autoTriggers = new AutoTriggers(trajectory, swerve::getPose, timer::get);

        autoTriggers.autoEnabled().whileTrue(
                Commands.sequence(
                        resetPose(trajectory.getInitialState()),
                        shoot().withName("ShootPreload").asProxy(),
                        followPath(trajectoryGroup.get(0), timer)
                ).withName("PreloadAndFollow0")
        );

        autoTriggers.atTime(1.7).onTrue(
                Commands.parallel(
                        intake.intakeCommand(),
                        shootCommands.readySuperstructureForShot()
                ).withName("Intake0")
        );

        autoTriggers.atTime(4.98).onTrue(
                Commands.sequence(
                        shootCommands.deferredStopAimAndShoot().withName("Shoot0").asProxy(),
                        followPath(trajectoryGroup.get(1), timer).asProxy()
                ).withName("Shoot0AndFollow1")
        );

        autoTriggers.atTime(6).onTrue(
                Commands.parallel(
                        intake.intakeCommand(),
                        shootCommands.readySuperstructureForShot()
                ).withName("Intake1")
        );

        autoTriggers.atTime(9.13).onTrue(
                Commands.sequence(
                        shootCommands.deferredStopAimAndShoot().withName("Shoot1").asProxy()
                ).withName("Shoot1AndFollow2")
        );

        return autoTriggers.eventLoop;
    }

    public EventLoop ampCenter3_2() {
        final String trajectoryName = "AmpCenter3_2";
        final ChoreoTrajectory trajectory = Choreo.getTrajectory(trajectoryName);
        final List<ChoreoTrajectory> trajectoryGroup = Choreo.getTrajectoryGroup(trajectoryName);

        final Timer timer = new Timer();
        final AutoTriggers autoTriggers = new AutoTriggers(trajectory, swerve::getPose, timer::get);

        autoTriggers.autoEnabled().whileTrue(
                Commands.sequence(
                        resetPose(trajectory.getInitialState()),
                        shoot().withName("ShootPreload").asProxy(),
                        followPath(trajectoryGroup.get(0), timer)
                ).withName("PreloadAndFollow0")
        );

        autoTriggers.autoEnabled().whileTrue(
                Commands.run(() -> Logger.recordOutput("Auto/Timer", timer.get()))
        );

        autoTriggers.atTime(2).onTrue(
                Commands.parallel(
                        intake.intakeCommand(),
                        shootCommands.readySuperstructureForShot()
                ).withName("Intake0")
        );

        autoTriggers.atTime(4.98).onTrue(
                Commands.sequence(
                        shootCommands.deferredStopAimAndShoot().withName("Shoot0").asProxy(),
                        followPath(trajectoryGroup.get(1), timer).asProxy()
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
        final String trajectoryName = "AmpCenter4_3";
        final ChoreoTrajectory trajectory = Choreo.getTrajectory(trajectoryName);
        final List<ChoreoTrajectory> trajectoryGroup = Choreo.getTrajectoryGroup(trajectoryName);

        final Timer timer = new Timer();
        final AutoTriggers autoTriggers = new AutoTriggers(trajectory, swerve::getPose, timer::get);

        autoTriggers.autoEnabled().whileTrue(
                Commands.sequence(
                        resetPose(trajectory.getInitialState()),
                        shoot().withName("ShootPreload").asProxy(),
                        followPath(trajectoryGroup.get(0), timer)
                ).withName("PreloadAndFollow0")
        );

        autoTriggers.autoEnabled().whileTrue(
                Commands.run(() -> Logger.recordOutput("Auto/Timer", timer.get()))
        );

        autoTriggers.atTime(2).onTrue(
                Commands.parallel(
                        intake.intakeCommand(),
                        shootCommands.readySuperstructureForShot()
                ).withName("Intake0")
        );

        autoTriggers.atTime(4.86).onTrue(
                Commands.sequence(
                        shootCommands.deferredStopAimAndShoot().withName("Shoot0").asProxy(),
                        followPath(trajectoryGroup.get(1), timer).asProxy()
                ).withName("Shoot0AndFollow1")
        );

        autoTriggers.atTime(5.5).onTrue(
                Commands.parallel(
                        intake.intakeCommand(),
                        shootCommands.readySuperstructureForShot()
                ).withName("Intake1")
        );

        autoTriggers.atTime(8.74).onTrue(
                Commands.sequence(
                        shootCommands.deferredStopAimAndShoot().withName("Shoot1").asProxy()
                ).withName("Shoot1AndFollow2")
        );

        return autoTriggers.eventLoop;
    }

    public EventLoop speaker2_1_0Center4_3() {
        final String trajectoryName = "Speaker2_1_0Center4_3";
        final ChoreoTrajectory trajectory = Choreo.getTrajectory(trajectoryName);
        final List<ChoreoTrajectory> trajectoryGroup = Choreo.getTrajectoryGroup(trajectoryName);

        final Timer timer = new Timer();
        final AutoTriggers autoTriggers = new AutoTriggers(trajectory, swerve::getPose, timer::get);

        autoTriggers.autoEnabled().whileTrue(
                Commands.sequence(
                        resetPose(trajectory.getInitialState()),
                        shoot().withName("ShootPreload").asProxy(),
                        followPath(trajectoryGroup.get(0), timer)
                ).withName("PreloadAndFollow0")
        );

        autoTriggers.atTime(0.3).onTrue(
                Commands.parallel(
                        intake.intakeCommand()
                ).withName("Intake0")
        );

        autoTriggers.atTime(1.6).onTrue(
                Commands.parallel(
                        shootCommands.shoot()
                ).withName("Shoot0")
        );

        autoTriggers.atTime(3.8).onTrue(
                Commands.parallel(
                        intake.intakeCommand(),
                        shootCommands.readySuperstructureForShot()
                ).withName("Intake1")
        );

        autoTriggers.atTime(6.15).onTrue(
                Commands.sequence(
                        shootCommands.deferredStopAimAndShoot().withName("Shoot1").asProxy(),
                        followPath(trajectoryGroup.get(1), timer).asProxy()
                ).withName("Shoot1AndFollow2")
        );

        autoTriggers.atTime(6.7).onTrue(
                Commands.parallel(
                        intake.intakeCommand(),
                        shootCommands.readySuperstructureForShot()
                ).withName("Intake2")
        );

        autoTriggers.atTime(9.01).onTrue(
                Commands.sequence(
                        shootCommands.deferredStopAimAndShoot().withName("Shoot2").asProxy(),
                        followPath(trajectoryGroup.get(2), timer).asProxy()
                ).withName("Shoot2AndFollow3")
        );

        autoTriggers.atTime(9.7).onTrue(
                Commands.parallel(
                        intake.intakeCommand(),
                        superstructure.toGoal(Superstructure.Goal.IDLE)
                ).withName("Intake3")
        );

        autoTriggers.atTime(12.27).onTrue(
                Commands.sequence(
                        shootCommands.deferredStopAimAndShoot().withName("Shoot3").asProxy()
                ).withName("Shoot3")
        );

        return autoTriggers.eventLoop;
    }
}
