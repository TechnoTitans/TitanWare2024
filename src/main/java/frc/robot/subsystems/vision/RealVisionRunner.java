package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.constants.Constants;
import frc.robot.utils.closeables.ToClose;
import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;

import java.util.HashMap;
import java.util.Map;

public class RealVisionRunner implements PhotonVisionRunner {
    public static class VisionIOApriltagsReal implements VisionIO {
        private final TitanCamera titanCamera;
        private final PhotonCamera photonCamera;
        private final String cameraName;
        private final double stdDevFactor;

        public VisionIOApriltagsReal(final TitanCamera titanCamera) {
            this.titanCamera = titanCamera;
            this.photonCamera = titanCamera.getPhotonCamera();
            this.stdDevFactor = titanCamera.getStdDevFactor();
            this.cameraName = photonCamera.getName();
        }

        @Override
        public void updateInputs(final VisionIO.VisionIOInputs inputs) {
            inputs.name = cameraName;
            inputs.stdDevFactor = stdDevFactor;
            inputs.latestResult = photonCamera.getLatestResult();
        }
    }

    private final Map<VisionIOApriltagsReal, VisionIO.VisionIOInputs> visionIOInputsMap;
    private final Map<VisionIOApriltagsReal, PhotonPoseEstimator> photonPoseEstimatorMap;

    private final Map<VisionIO, EstimatedRobotPose> estimatedRobotPoseMap;

    public RealVisionRunner(
            final AprilTagFieldLayout aprilTagFieldLayout,
            final Map<VisionIOApriltagsReal, VisionIO.VisionIOInputs> photonVisionIOInputsMap
    ) {
        this.visionIOInputsMap = photonVisionIOInputsMap;

        final Map<VisionIOApriltagsReal, PhotonPoseEstimator> poseEstimatorMap = new HashMap<>();
        for (final VisionIOApriltagsReal visionIOApriltagsReal : visionIOInputsMap.keySet()) {
            final PhotonPoseEstimator photonPoseEstimator = new PhotonPoseEstimator(
                    aprilTagFieldLayout,
                    Constants.Vision.MULTI_TAG_POSE_STRATEGY,
                    visionIOApriltagsReal.photonCamera,
                    visionIOApriltagsReal.titanCamera.getRobotRelativeToCameraTransform()
            );
            photonPoseEstimator.setMultiTagFallbackStrategy(PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY);

            poseEstimatorMap.put(visionIOApriltagsReal, photonPoseEstimator);
        }

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
                final Map.Entry<VisionIOApriltagsReal, VisionIO.VisionIOInputs>
                        visionIOInputsEntry : visionIOInputsMap.entrySet()
        ) {
            final VisionIOApriltagsReal visionIO = visionIOInputsEntry.getKey();
            final VisionIO.VisionIOInputs inputs = visionIOInputsEntry.getValue();

            visionIO.periodic();
            visionIO.updateInputs(inputs);

            Logger.processInputs(
                    String.format("%s/%s", PhotonVision.PhotonLogKey, inputs.name),
                    inputs
            );

            VisionUtils.updatePoseEstimator(
                    photonPoseEstimatorMap.get(visionIO),
                    inputs.latestResult
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
    public Map<VisionIOApriltagsReal, VisionIO.VisionIOInputs> getVisionIOInputsMap() {
        return visionIOInputsMap;
    }

    @Override
    public EstimatedRobotPose getEstimatedRobotPose(final VisionIO visionIO) {
        return estimatedRobotPoseMap.get(visionIO);
    }
}
