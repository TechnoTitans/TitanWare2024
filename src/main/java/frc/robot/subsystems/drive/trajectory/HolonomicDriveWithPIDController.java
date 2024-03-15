package frc.robot.subsystems.drive.trajectory;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class HolonomicDriveWithPIDController {
    private final PIDController xController;
    private final PIDController yController;
    private final ProfiledPIDController rotationController;

    private final Pose2d poseTolerance;
    private Pose2d poseError = new Pose2d();
    private Rotation2d rotationError = new Rotation2d();

    /**
     * Constructs a {@link HolonomicDriveWithPIDController}
     *
     * @param xController        A {@link PIDController} to respond to error in the field-relative X direction
     * @param yController        A {@link PIDController} to respond to error in the field-relative Y direction
     * @param rotationController A {@link ProfiledPIDController} controller to respond to error in rotation
     */
    public HolonomicDriveWithPIDController(
            final PIDController xController,
            final PIDController yController,
            final ProfiledPIDController rotationController,
            final Pose2d poseTolerance
    ) {
        this.xController = xController;
        this.yController = yController;
        this.rotationController = rotationController;
        this.rotationController.enableContinuousInput(-Math.PI, Math.PI);

        this.poseTolerance = poseTolerance;
    }

    /**
     * Resets the state of the DriveController by resetting accumulated error on all PID controllers
     * @see PIDController#reset()
     * @see ProfiledPIDController#reset(double, double)
     */
    public void reset(final Pose2d currentPose, final ChassisSpeeds robotRelativeSpeeds) {
        xController.reset();
        yController.reset();
        rotationController.reset(currentPose.getRotation().getRadians(), robotRelativeSpeeds.omegaRadiansPerSecond);
    }

    /**
     * Returns true if the pose error is within tolerance of the reference.
     *
     * @return True if the pose error is within tolerance of the reference.
     */
    public boolean atReference() {
        final Translation2d translationError = poseError.getTranslation();
        final Translation2d translationTolerance = poseTolerance.getTranslation();
        final Rotation2d rotationTolerance = poseTolerance.getRotation();

        return Math.abs(translationError.getX()) < translationTolerance.getX()
                && Math.abs(translationError.getY()) < translationTolerance.getY()
                && Math.abs(rotationError.getRadians()) < rotationTolerance.getRadians();
    }

    /**
     * Calculates the next output of the holonomic drive controller
     *
     * @param currentPose The current {@link Pose2d}
     * @param targetPose The desired {@link Pose2d}
     * @return The next output of the holonomic drive controller
     */
    public ChassisSpeeds calculate(final Pose2d currentPose, final Pose2d targetPose) {
        final double xFeedback = xController.calculate(currentPose.getX(), targetPose.getX());
        final double yFeedback = yController.calculate(currentPose.getY(), targetPose.getY());
        final double rotationFeedback = rotationController.calculate(
                currentPose.getRotation().getRadians(),
                targetPose.getRotation().getRadians()
        );

        this.poseError = targetPose.relativeTo(currentPose);
        this.rotationError = targetPose.getRotation().minus(currentPose.getRotation());

        return ChassisSpeeds.fromFieldRelativeSpeeds(
                xFeedback,
                yFeedback,
                rotationFeedback,
                currentPose.getRotation()
        );
    }
}
