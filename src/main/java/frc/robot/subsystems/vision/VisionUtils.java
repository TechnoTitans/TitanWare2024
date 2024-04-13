package frc.robot.subsystems.vision;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.constants.Constants;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.MultiTargetPNPResult;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.Optional;

public class VisionUtils {
    public static void correctPipelineResultTimestamp(final PhotonPipelineResult photonPipelineResult) {
        final double currentTimestamp = Timer.getFPGATimestamp();
        if (photonPipelineResult.getTimestampSeconds() > currentTimestamp) {
            // if result in the future, use alternative timestamp calculated from latency instead
            photonPipelineResult.setTimestampSeconds(
                    currentTimestamp - Units.millisecondsToSeconds(photonPipelineResult.getLatencyMillis())
            );
        }
    }

    public static Optional<EstimatedRobotPose> updatePoseEstimator(
            final PhotonPoseEstimator photonPoseEstimator,
            final PhotonPipelineResult photonPipelineResult
    ) {
        if (!photonPipelineResult.hasTargets()) {
            return Optional.empty();
        }

        final int nTargets = photonPipelineResult.targets.size();
        final MultiTargetPNPResult multiTargetPNPResult = photonPipelineResult.getMultiTagResult();
        if (multiTargetPNPResult.estimatedPose.isPresent) {
            // multi-tag
            // TODO: use ambiguity to disambiguate using PNP alternate here instead
            if (multiTargetPNPResult.estimatedPose.ambiguity > Constants.Vision.MULTI_TAG_MAX_AMBIGUITY) {
                return Optional.empty();
            }
        } else if (nTargets == 1) {
            // single-tag
            // TODO: use ambiguity to disambiguate using altCameraToTarget here instead
            final PhotonTrackedTarget photonTrackedTarget = photonPipelineResult.targets.get(0);
            if (photonTrackedTarget.getPoseAmbiguity() > Constants.Vision.SINGLE_TAG_MAX_AMBIGUITY) {
                return Optional.empty();
            }
        } else {
            // no estimate
            return Optional.empty();
        }

        return photonPoseEstimator.update(photonPipelineResult);
    }
}
