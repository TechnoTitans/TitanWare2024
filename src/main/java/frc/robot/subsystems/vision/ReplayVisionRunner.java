package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.constants.Constants;
import frc.robot.subsystems.vision.cameras.TitanCamera;
import frc.robot.utils.closeables.ToClose;
import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;

import java.util.HashMap;
import java.util.Map;

public class ReplayVisionRunner implements PhotonVisionRunner {
    public static class VisionIOApriltagsReplay implements VisionIO {
        private final TitanCamera titanCamera;
        private final PhotonCamera photonCamera;

        public VisionIOApriltagsReplay(final TitanCamera titanCamera) {
            this.titanCamera = titanCamera;
            this.photonCamera = titanCamera.getPhotonCamera();
        }
    }

    private final Map<ReplayVisionRunner.VisionIOApriltagsReplay, String> visionIONames;
    private final Map<ReplayVisionRunner.VisionIOApriltagsReplay, VisionIO.VisionIOInputs> visionIOInputsMap;
    private final Map<ReplayVisionRunner.VisionIOApriltagsReplay, PhotonPoseEstimator> photonPoseEstimatorMap;

    private final Map<VisionIO, EstimatedRobotPose> estimatedRobotPoseMap;

    public ReplayVisionRunner(
            final AprilTagFieldLayout aprilTagFieldLayout,
            final Map<ReplayVisionRunner.VisionIOApriltagsReplay, VisionIO.VisionIOInputs> visionIOInputsMap
    ) {
        this.visionIOInputsMap = visionIOInputsMap;

        final Map<ReplayVisionRunner.VisionIOApriltagsReplay, String> visionIONames = new HashMap<>();
        final Map<ReplayVisionRunner.VisionIOApriltagsReplay, PhotonPoseEstimator> poseEstimatorMap = new HashMap<>();
        for (final ReplayVisionRunner.VisionIOApriltagsReplay visionIOApriltagsReplay : visionIOInputsMap.keySet()) {
            final PhotonPoseEstimator photonPoseEstimator = new PhotonPoseEstimator(
                    aprilTagFieldLayout,
                    Constants.Vision.MULTI_TAG_POSE_STRATEGY,
                    visionIOApriltagsReplay.photonCamera,
                    visionIOApriltagsReplay.titanCamera.getRobotToCameraTransform()
            );
            photonPoseEstimator.setMultiTagFallbackStrategy(PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY);

            poseEstimatorMap.put(visionIOApriltagsReplay, photonPoseEstimator);
            visionIONames.put(visionIOApriltagsReplay, visionIOApriltagsReplay.photonCamera.getName());
        }

        this.visionIONames = visionIONames;
        this.photonPoseEstimatorMap = poseEstimatorMap;
        this.estimatedRobotPoseMap = new HashMap<>();
    }

    @SuppressWarnings("DuplicatedCode")
    @Override
    public void periodic() {
        if (ToClose.hasClosed()) {
            return;
        }

        for (
                final Map.Entry<ReplayVisionRunner.VisionIOApriltagsReplay, VisionIO.VisionIOInputs>
                        photonVisionIOInputsEntry : visionIOInputsMap.entrySet()
        ) {
            final ReplayVisionRunner.VisionIOApriltagsReplay visionIO = photonVisionIOInputsEntry.getKey();
            final VisionIO.VisionIOInputs inputs = photonVisionIOInputsEntry.getValue();

            visionIO.periodic();
            visionIO.updateInputs(inputs);

            Logger.processInputs(
                    String.format("%s/%s", PhotonVision.PhotonLogKey, visionIONames.get(visionIO)),
                    inputs
            );

            final PhotonPipelineResult result = inputs.latestResult;
            VisionUtils.correctPipelineResultTimestamp(result);
            VisionUtils.updatePoseEstimator(
                    photonPoseEstimatorMap.get(visionIO),
                    result
            ).ifPresent(
                    estimatedRobotPose -> estimatedRobotPoseMap.put(visionIO, estimatedRobotPose)
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
    public Map<ReplayVisionRunner.VisionIOApriltagsReplay, VisionIO.VisionIOInputs> getApriltagVisionIOInputsMap() {
        return visionIOInputsMap;
    }

    @Override
    public EstimatedRobotPose getEstimatedRobotPose(final VisionIO visionIO) {
        return estimatedRobotPoseMap.get(visionIO);
    }
}
