package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.constants.Constants;
import frc.robot.utils.closeables.ToClose;
import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.List;
import java.util.Map;
import java.util.Optional;

public class PhotonVisionApriltagsReal implements PhotonVisionRunner<PhotonVisionApriltagsReal.IOApriltagsReal> {
    public static class IOApriltagsReal implements PhotonVisionIO {
        private final PhotonCamera photonCamera;
        private final String logKey;

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
            this.logKey = String.format("%s_PhotonVisionIOApriltagsReal", photonCamera.getName());

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

        @Override
        public void updateInputs(final PhotonVisionIOInputs inputs) {
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

            final List<PhotonTrackedTarget> targets = photonResult.targets;
            final int nTargets = targets.size();

            if (nTargets == 1) {
                // single-tag
                final PhotonTrackedTarget firstTarget = targets.get(0);
                if (firstTarget.getPoseAmbiguity() <= Constants.Vision.SINGLE_TAG_MAX_AMBIGUITY) {
                    updatePoseEstimator(photonResult);
                }
            } else {
                // multi-tag
                final List<PhotonTrackedTarget> filteredTargets = photonResult.getTargets()
                        .stream()
                        .filter(
                                photonTrackedTarget ->
                                        photonTrackedTarget.getPoseAmbiguity() <= Constants.Vision.MULTI_TAG_MAX_AMBIGUITY
                        )
                        .toList();

                final PhotonPipelineResult filteredPhotonPipelineResult = new PhotonPipelineResult(
                        photonResult.getLatencyMillis(), filteredTargets
                );

                filteredPhotonPipelineResult.setTimestampSeconds(photonResult.getTimestampSeconds());
                updatePoseEstimator(filteredPhotonPipelineResult);
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
                    String.format("%s/%s", PhotonVision.photonLogKey, ioApriltagsReal.logKey),
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
