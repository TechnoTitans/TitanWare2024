package frc.robot.subsystems.drive.trajectory;

import com.choreo.lib.ChoreoTrajectoryState;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class HolonomicChoreoController {
    private final PIDController xController;
    private final PIDController yController;
    private final PIDController rotationController;

    public HolonomicChoreoController(
            final PIDController xController,
            final PIDController yController,
            final PIDController rotationController
    ) {
        this.xController = xController;
        this.yController = yController;
        this.rotationController = rotationController;

        this.rotationController.enableContinuousInput(-Math.PI, Math.PI);
    }

    public void reset() {
        xController.reset();
        yController.reset();
        rotationController.reset();
    }

    public ChassisSpeeds calculate(
            final Pose2d pose,
            final ChoreoTrajectoryState referenceState
    ) {
        double xFF = referenceState.velocityX;
        double yFF = referenceState.velocityY;
        double rotationFF = referenceState.angularVelocity;

        double xFeedback = xController.calculate(pose.getX(), referenceState.x);
        double yFeedback = yController.calculate(pose.getY(), referenceState.y);
        double rotationFeedback =
                rotationController.calculate(pose.getRotation().getRadians(), referenceState.heading);

        return ChassisSpeeds.fromFieldRelativeSpeeds(
                xFF + xFeedback,
                yFF + yFeedback,
                rotationFF + rotationFeedback,
                pose.getRotation()
        );
    }

    public ChassisSpeeds calculate(
            final Pose2d pose,
            final ChoreoTrajectoryState referenceState,
            final Rotation2d headingOverride
    ) {
        double xFF = referenceState.velocityX;
        double yFF = referenceState.velocityY;

        double xFeedback = xController.calculate(pose.getX(), referenceState.x);
        double yFeedback = yController.calculate(pose.getY(), referenceState.y);
        double rotationFeedback =
                rotationController.calculate(
                        pose.getRotation().getRadians(),
                        headingOverride.getRadians()
                );

        return ChassisSpeeds.fromFieldRelativeSpeeds(
                xFF + xFeedback,
                yFF + yFeedback,
                rotationFeedback,
                pose.getRotation()
        );
    }
}
