package frc.robot.auto;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.drive.Swerve;

import java.util.Optional;
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

    private static BooleanSupplier placeAndTime(
            final Supplier<Pose2d> currentPoseSupplier,
            final DoubleSupplier timeSupplier,
            final Translation2d place,
            final double translationToleranceMeters,
            final double timeSeconds,
            final double timeToleranceSeconds
    ) {
        return () -> currentPoseSupplier
                .get()
                .getTranslation()
                .getDistance(place) < translationToleranceMeters
                && Math.abs(timeSupplier.getAsDouble() - timeSeconds) < timeToleranceSeconds;
    }

    private static BooleanSupplier placeAndTime(
            final Supplier<Pose2d> currentPoseSupplier,
            final DoubleSupplier timeSupplier,
            final ChoreoTrajectory choreoTrajectory,
            final BooleanSupplier mirrorForRedAllianceSupplier,
            final double timeSeconds,
            final double timeToleranceSeconds,
            final double translationToleranceMeters
    ) {
        final boolean mirrorForRedAlliance = mirrorForRedAllianceSupplier.getAsBoolean();
        return Autos.placeAndTime(
                currentPoseSupplier,
                timeSupplier,
                choreoTrajectory.sample(timeSeconds, mirrorForRedAlliance)
                        .getPose()
                        .getTranslation(),
                translationToleranceMeters,
                timeSeconds,
                timeToleranceSeconds
        );
    }

    private static BooleanSupplier placeAndTime(
            final Supplier<Pose2d> currentPoseSupplier,
            final DoubleSupplier timeSupplier,
            final ChoreoTrajectory choreoTrajectory,
            final double timeSeconds
    ) {
        return Autos.placeAndTime(
                currentPoseSupplier,
                timeSupplier,
                choreoTrajectory,
                IsRedAlliance,
                timeSeconds,
                TimeToleranceSeconds,
                TranslationToleranceMeters
        );
    }

    private Trigger atPlaceAndTime(
            final EventLoop eventLoop,
            final DoubleSupplier timeSupplier,
            final ChoreoTrajectory choreoTrajectory,
            final double timeSeconds
    ) {
        return new Trigger(
                eventLoop,
                Autos.placeAndTime(swerve::getPose, timeSupplier, choreoTrajectory, timeSeconds)
        );
    }

    private Command followPath(final ChoreoTrajectory choreoTrajectory) {
        return swerve.followChoreoPathCommand(choreoTrajectory);
    }

    public Command sourceBoth() {
        final EventLoop eventLoop = new EventLoop();
        final Timer timer = new Timer();

        final ChoreoTrajectory sourceBoth = Choreo.getTrajectory("SourceBoth");

        atPlaceAndTime(
                eventLoop,
                timer::get,
                sourceBoth,
                0
        ).onTrue(
                Commands.print("shoot preload")
        );

        atPlaceAndTime(
                eventLoop,
                timer::get,
                sourceBoth,
                0.27
        ).onTrue(
                Commands.print("run intake for close 0")
        );

        return Commands.parallel(
                Commands.runOnce(timer::restart),
                Commands.sequence(
                        swerve.resetPoseCommand(sourceBoth.getInitialPose()),
                        followPath(sourceBoth)
                ).deadlineWith(Commands.run(eventLoop::poll)
        ));
    }
}
