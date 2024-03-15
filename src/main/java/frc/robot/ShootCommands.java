package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.drive.Swerve;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.superstructure.ShotParameters;
import frc.robot.subsystems.superstructure.Superstructure;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class ShootCommands {
    private final Swerve swerve;
    private final Intake intake;
    private final Superstructure superstructure;

    public ShootCommands(final Swerve swerve, final Intake intake, final Superstructure superstructure) {
        this.swerve = swerve;
        this.intake = intake;
        this.superstructure = superstructure;
    }

    public static Supplier<Rotation2d> angleToSpeakerSupplier(final Supplier<Pose2d> currentPoseSupplier) {
        return () -> currentPoseSupplier.get()
                .getTranslation()
                .minus(FieldConstants.getSpeakerPose().getTranslation())
                .getAngle()
                .minus(Rotation2d.fromRadians(Math.PI));
    }

    public static Supplier<ShotParameters.Parameters> shotParametersSupplier(
            final Supplier<Pose2d> currentPoseSupplier
    ) {
        return () -> ShotParameters.get(
                currentPoseSupplier.get()
                        .minus(FieldConstants.getSpeakerPose())
                        .getTranslation()
                        .getNorm()
        );
    }

    public Command amp() {
        return Commands.sequence(
                intake.feedHalfCommand(),
                Commands.deadline(
                        Commands.waitUntil(superstructure.atSetpoint)
                                .andThen(Commands.waitSeconds(0.5))
                                .andThen(intake.feedCommand())
                                .andThen(Commands.waitSeconds(0.5)),
                        superstructure.toGoal(Superstructure.Goal.AMP)
                )
        );
    }

    public Command lineupAndAmp() {
        return Commands.deadline(
                Commands.sequence(
                        Commands.waitUntil(swerve.atHolonomicDrivePose)
                                .withTimeout(6),
                        amp()
                ),
                swerve.driveToPose(FieldConstants::getAmpScoringPose)
        );
    }

    public Command shootSubwoofer() {
        return Commands.deadline(
                intake
                        .runStopCommand()
                        .until(superstructure.atSetpoint)
                        .andThen(intake.feedCommand()),
                superstructure.toGoal(Superstructure.Goal.SUBWOOFER)
        );
    }

    public Command stopAimAndShoot() {
        return Commands.deadline(
                Commands.deadline(
                        intake
                                .runStopCommand()
                                .until(superstructure.atSetpoint.and(swerve.atHeadingSetpoint))
                                .andThen(intake.feedCommand()),
                        superstructure.runState(ShootCommands.shotParametersSupplier(swerve::getPose))
                ),
                swerve.faceAngle(ShootCommands.angleToSpeakerSupplier(swerve::getPose))
        );
    }

    public Command teleopDriveAimAndShoot(
            final DoubleSupplier xStickInput,
            final DoubleSupplier yStickInput
    ) {
        return Commands.deadline(
                Commands.deadline(
                        intake
                                .runStopCommand()
                                .until(superstructure.atSetpoint.and(swerve.atHeadingSetpoint))
                                .andThen(intake.feedCommand()),
                        superstructure.runState(ShootCommands.shotParametersSupplier(swerve::getPose))
                ),
                swerve.teleopFacingAngleCommand(
                        xStickInput,
                        yStickInput,
                        ShootCommands.angleToSpeakerSupplier(swerve::getPose)
                )
        );
    }
}
