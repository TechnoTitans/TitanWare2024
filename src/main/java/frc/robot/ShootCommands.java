package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.drive.Swerve;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.superstructure.ShootOnTheMove;
import frc.robot.subsystems.superstructure.ShotParameters;
import frc.robot.subsystems.superstructure.Superstructure;

import java.util.Set;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

@SuppressWarnings("unused")
public class ShootCommands {
    private final Swerve swerve;
    private final Intake intake;
    private final Superstructure superstructure;

    public ShootCommands(final Swerve swerve, final Intake intake, final Superstructure superstructure) {
        this.swerve = swerve;
        this.intake = intake;
        this.superstructure = superstructure;
    }

    public static Rotation2d angleToPose(final Pose2d currentPose, final Pose2d angleTowardsPose) {
        return currentPose
                .getTranslation()
                .minus(angleTowardsPose.getTranslation())
                .getAngle()
                .minus(Rotation2d.fromRadians(Math.PI));
    }

    public static Rotation2d angleToSpeaker(final Pose2d currentPose) {
        return angleToPose(currentPose, FieldConstants.getSpeakerPose());
    }

    public static Supplier<Rotation2d> angleToSpeakerSupplier(final Supplier<Pose2d> currentPoseSupplier) {
        return () -> angleToSpeaker(currentPoseSupplier.get());
    }

    public static Rotation2d angleToFerry(final Pose2d currentPose) {
        return angleToPose(currentPose, FieldConstants.getAmpScoringPose());
    }

    public static Supplier<Rotation2d> angleToFerrySupplier(final Supplier<Pose2d> currentPoseSupplier) {
        return () -> angleToFerry(currentPoseSupplier.get());
    }

    public static ShootOnTheMove.Shot shotWhileMoving(
            final Pose2d currentPose,
            final ChassisSpeeds chassisSpeeds
    ) {
        return ShootOnTheMove.calculate(
                currentPose,
                chassisSpeeds,
                ShotParameters::getShotParameters
        );
    }

    public static Supplier<ShootOnTheMove.Shot> shotWhileMovingSupplier(
            final Supplier<Pose2d> currentPoseSupplier,
            final Supplier<ChassisSpeeds> chassisSpeedsSupplier
    ) {
        return () -> ShootOnTheMove.calculate(
                currentPoseSupplier.get(),
                chassisSpeedsSupplier.get(),
                ShotParameters::getShotParameters
        );
    }

    public Command runEjectShooter() {
        return Commands.parallel(
                intake.runEjectInCommand(),
                superstructure.toGoal(Superstructure.Goal.EJECT)
        );
    }

    public Command runEjectIntake() {
        return Commands.deadline(
                intake.runEjectOutCommand(),
                superstructure.toGoal(Superstructure.Goal.BACK_FEED)
        );
    }

    public Command readyAmp() {
        return Commands.sequence(
                intake.feedHalfCommand(),
                superstructure.runGoal(Superstructure.Goal.AMP)
        );
    }

    public Command amp() {
        return Commands.sequence(
                intake.feedHalfCommand()
                        .onlyIf(() -> superstructure.getGoal() != Superstructure.Goal.AMP),
                Commands.deadline(
                        Commands.waitUntil(superstructure.atSetpoint)
                                .andThen(Commands.waitSeconds(0.1))
                                .andThen(intake.feedCommand())
                                .andThen(Commands.waitSeconds(0.4)),
                        superstructure.toGoal(Superstructure.Goal.AMP)
                )
        );
    }

    public Command angleAndReadyAmp(
            final DoubleSupplier xStickInput,
            final DoubleSupplier yStickInput
    ) {
        return Commands.parallel(
                swerve.teleopFacingAngleCommand(
                        xStickInput,
                        yStickInput,
                        () -> FieldConstants.getAmpScoringPose().getRotation()
                ),
                readyAmp()
        );
    }

    public Command lineupAndAmp() {
        return swerve.driveToPose(FieldConstants::getAmpScoringPose)
                .until(swerve.atHolonomicDrivePose)
                .withTimeout(6)
                .andThen(Commands.deadline(
                        Commands.sequence(
                                Commands.waitSeconds(0.2),
                                amp()
                        ),
                        swerve.teleopDriveCommand(() -> 0, () -> 0.5, () -> 0)
                ));
    }

    public Command shootSubwoofer() {
        return Commands.deadline(
                intake.runStopCommand()
                        .until(superstructure.atSetpoint)
                        .andThen(intake.feedCommand()),
//                Commands.waitSeconds(0.2)
//                        .andThen(intake.feedCommand()),
                superstructure.toGoal(Superstructure.Goal.SUBWOOFER)
        );
    }

    public Command ferryCenterToAmp() {
        return Commands.deadline(
                Commands.deadline(
                        intake
                                .runStopCommand()
                                .until(superstructure.atSetpoint.and(swerve.atHeadingSetpoint))
                                .withTimeout(1.5) // TODO: fixme
                                .andThen(intake.feedCommand()),
                        superstructure.toGoal(Superstructure.Goal.FERRY_CENTERLINE)
                ),
                swerve.faceAngle(() -> ShootCommands.angleToPose(
                        swerve.getPose(),
                        FieldConstants.getAmpScoringPose())
                )
        );
    }

    public Command readyFerry(
            final DoubleSupplier xStickInput,
            final DoubleSupplier yStickInput
    ) {
        final Supplier<ShotParameters.Parameters> ferrySupplier =
                () -> ShotParameters.getFerryParameters(swerve.getPose());

        return Commands.parallel(
                superstructure.toState(ferrySupplier),
                swerve.teleopFacingAngleCommand(
                        xStickInput,
                        yStickInput,
                        ShootCommands.angleToFerrySupplier(swerve::getPose)
                )
        );
    }

    public Command ferry() {
        return Commands.deadline(
                Commands.deadline(
                        intake
                                .runStopCommand()
                                .until(superstructure.atSetpoint.and(swerve.atHeadingSetpoint))
                                .andThen(intake.feedCommand()),
                        superstructure.toState(() -> ShotParameters.getFerryParameters(swerve.getPose()))
                ),
                swerve.faceAngle(ShootCommands.angleToFerrySupplier(swerve::getPose))
        );
    }

    public Command readySuperstructureForShot() {
        return superstructure.toState(ShotParameters.shotParametersSupplier(swerve::getPose));
    }

    public Command shoot() {
        return Commands.deadline(
                intake
                        .runStopCommand()
                        .until(superstructure.atSetpoint)
                        .withTimeout(1) // TODO: fixme
                        .andThen(intake.feedCommand()),
                superstructure.toState(ShotParameters.shotParametersSupplier(swerve::getPose))
        );
    }

    public Command stopAimAndShoot() {
        return Commands.deadline(
                Commands.deadline(
                        intake
                                .runStopCommand()
                                .until(superstructure.atSetpoint.and(swerve.atHeadingSetpoint))
                                .withTimeout(1.5) // TODO: fixme
                                .andThen(intake.feedCommand()),
                        superstructure.toState(ShotParameters.shotParametersSupplier(swerve::getPose))
                ),
                swerve.faceAngle(ShootCommands.angleToSpeakerSupplier(swerve::getPose))
        );
    }

    public Command deferredStopAimAndShoot() {
        return Commands.deadline(
                Commands.deadline(
                        intake
                                .runStopCommand()
                                .until(superstructure.atSetpoint.and(swerve.atHeadingSetpoint))
                                .withTimeout(1.5) // TODO: fixme
                                .andThen(intake.feedCommand()),
                        Commands.defer(() ->
                                        superstructure.toState(() -> ShotParameters.getShotParameters(swerve.getPose())),
                                superstructure.getRequirements()
                        )
                ),
                Commands.defer(
                        () -> swerve.faceAngle(() -> ShootCommands.angleToSpeaker(swerve.getPose())),
                        Set.of(swerve)
                )
        );
    }

    @SuppressWarnings("unused")
    public Command teleopDriveAimAndShoot(
            final DoubleSupplier xStickInput,
            final DoubleSupplier yStickInput
    ) {
        final Supplier<ShootOnTheMove.Shot> shotSupplier = ShootCommands.shotWhileMovingSupplier(
                () -> {
                    final Pose2d currentPose = swerve.getPose();
                    return new Pose2d(currentPose.getTranslation(), ShootCommands.angleToSpeaker(currentPose));
                },
                swerve::getRobotRelativeSpeeds
        );

        return Commands.deadline(
                Commands.deadline(
                        intake
                                .runStopCommand()
                                .until(superstructure.atSetpoint.and(swerve.atHeadingSetpoint))
//                                .withTimeout(1.5)
                                .andThen(intake.feedCommand()),
                        superstructure.toState(() -> shotSupplier.get().parameters())
                ),
                swerve.teleopFacingAngleCommand(
                        xStickInput,
                        yStickInput,
                        () -> shotSupplier.get().futurePose().getRotation()
                )
        );
    }

    public Command readyShot(
            final DoubleSupplier xStickInput,
            final DoubleSupplier yStickInput
    ) {
        return Commands.parallel(
                intake.runStopCommand(),
                superstructure.toState(ShotParameters.shotParametersSupplier(swerve::getPose)),
                swerve.teleopFacingAngleCommand(
                        xStickInput,
                        yStickInput,
                        ShootCommands.angleToSpeakerSupplier(swerve::getPose)
                )
        );
    }
}
