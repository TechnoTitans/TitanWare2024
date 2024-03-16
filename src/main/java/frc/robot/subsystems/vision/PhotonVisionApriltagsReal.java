package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.constants.Constants;
import frc.robot.utils.closeables.ToClose;
import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.MultiTargetPNPResult;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.Map;
import java.util.Optional;

@SuppressWarnings("ClassCanBeRecord")
public class PhotonVisionApriltagsReal implements PhotonVisionRunner<PhotonVisionApriltagsReal.IOApriltagsReal> {
    public static class IOApriltagsReal implements PhotonVisionIO {
        private final PhotonCamera photonCamera;
        private final String cameraName;

        private final PhotonPoseEstimator poseEstimator;

        private PhotonPipelineResult latestPhotonPipelineResult;
        private EstimatedRobotPose estimatedRobotPose;
        /**
         * A stable {@link EstimatedRobotPose}, this does NOT get set to null after we get
         * the estimated pose, thus, it is stable and represents the last estimated pose.
         * <p>
         * Do <b>NOT</b> use this {@link EstimatedRobotPose} to feed into vision adjustments.
         */
        private EstimatedRobotPose stableEstimatedRobotPose;

        public IOApriltagsReal(
                final TitanCamera titanCamera,
                final AprilTagFieldLayout blueSideApriltagFieldLayout
        ) {
            this.photonCamera = titanCamera.getPhotonCamera();
            this.cameraName = photonCamera.getName();

            this.poseEstimator = new PhotonPoseEstimator(
                    blueSideApriltagFieldLayout,
                    Constants.Vision.MULTI_TAG_POSE_STRATEGY,
                    photonCamera,
                    titanCamera.getRobotRelativeToCameraTransform()
            );
            this.poseEstimator.setMultiTagFallbackStrategy(PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY);
        }

        private void updatePoseEstimator(final PhotonPipelineResult photonPipelineResult) {
            final Optional<EstimatedRobotPose> optionalEstimatedRobotPose = poseEstimator.update(photonPipelineResult);
            optionalEstimatedRobotPose.ifPresent(estimatedRobotPose -> {
                this.estimatedRobotPose = estimatedRobotPose;
                this.stableEstimatedRobotPose = estimatedRobotPose;
            });

            this.latestPhotonPipelineResult = photonPipelineResult;
        }

        @SuppressWarnings("DuplicatedCode")
        @Override
        public void updateInputs(final PhotonVisionIOInputs inputs) {
            inputs.name = cameraName;
            if (latestPhotonPipelineResult == null) {
                inputs.pipelineResultTargets = new double[] {};
            } else {
                inputs.pipelineResultTargets = latestPhotonPipelineResult.targets.stream()
                        .mapToDouble(PhotonTrackedTarget::getFiducialId)
                        .toArray();
            }
            inputs.primaryStrategy = poseEstimator.getPrimaryStrategy().toString();

            inputs.estimatedRobotPose = getLatestEstimatedPose();
            inputs.stableEstimatedRobotPose = getStableLastEstimatedPose();
        }

        @Override
        public void periodic() {
            final PhotonPipelineResult photonResult = photonCamera.getLatestResult();
            if (!photonResult.hasTargets()) {
                this.stableEstimatedRobotPose = null;
                return;
            }

            final MultiTargetPNPResult multiTargetPNPResult = photonResult.getMultiTagResult();
            if (multiTargetPNPResult.estimatedPose.isPresent) {
                // multi-tag
                if (multiTargetPNPResult.estimatedPose.ambiguity <= Constants.Vision.MULTI_TAG_MAX_AMBIGUITY) {
                    updatePoseEstimator(photonResult);
                }
            } else {
                // single-tag
                final PhotonTrackedTarget photonTrackedTarget = photonResult.targets.get(0);
                if (photonTrackedTarget.getPoseAmbiguity() <= Constants.Vision.SINGLE_TAG_MAX_AMBIGUITY) {
                    updatePoseEstimator(photonResult);
                }
            }
        }

        public EstimatedRobotPose getLatestEstimatedPose() {
            final EstimatedRobotPose robotPose = estimatedRobotPose;
            this.estimatedRobotPose = null;

            return robotPose;
        }

        public EstimatedRobotPose getStableLastEstimatedPose() {
            return stableEstimatedRobotPose;
        }
    }


    private final Map<IOApriltagsReal, PhotonVisionIO.PhotonVisionIOInputs>
            photonVisionIOInputsMap;

    public PhotonVisionApriltagsReal(
            final Map<IOApriltagsReal, PhotonVisionIO.PhotonVisionIOInputs>
                    photonVisionIOInputsMap
    ) {
        this.photonVisionIOInputsMap = photonVisionIOInputsMap;
    }

    @Override
    public void periodic() {
        if (ToClose.hasClosed()) {
            // do not try to update if we've already closed or if we cannot continue running
            return;
        }

        for (
                final Map.Entry<IOApriltagsReal, PhotonVisionIO.PhotonVisionIOInputs>
                        photonVisionIOInputsEntry : photonVisionIOInputsMap.entrySet()
        ) {
            final IOApriltagsReal ioApriltagsReal = photonVisionIOInputsEntry.getKey();
            final PhotonVisionIO.PhotonVisionIOInputs ioInputs = photonVisionIOInputsEntry.getValue();

            ioApriltagsReal.periodic();
            ioApriltagsReal.updateInputs(ioInputs);

            Logger.processInputs(
                    String.format("%s/%s", PhotonVision.PhotonLogKey, ioApriltagsReal.cameraName),
                    ioInputs
            );
        }
    }

    /**
     * Reset the simulated robot {@link Pose3d}.
     * @param robotPose the new robot {@link Pose3d}
     */
    @Override
    public void resetRobotPose(final Pose3d robotPose) {}

    @Override
    public Map<PhotonVisionApriltagsReal.IOApriltagsReal, PhotonVisionIO.PhotonVisionIOInputs> getPhotonVisionIOInputsMap() {
        return photonVisionIOInputsMap;
    }
}
