package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.constants.Constants;
import frc.robot.subsystems.vision.cameras.TitanCamera;
import frc.robot.subsystems.vision.result.NoteTrackingResult;
import frc.robot.utils.closeables.ToClose;
import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;

import java.util.HashMap;
import java.util.Map;

public class RealVisionRunner implements PhotonVisionRunner {
    public static class VisionIOApriltagReal implements VisionIO {
        private final TitanCamera titanCamera;
        private final PhotonCamera photonCamera;
        private final String cameraName;

        private final double stdDevFactor;
        private final Transform3d robotToCamera;

        public VisionIOApriltagReal(final TitanCamera titanCamera) {
            this.titanCamera = titanCamera;
            this.photonCamera = titanCamera.getPhotonCamera();
            this.cameraName = photonCamera.getName();

            this.stdDevFactor = titanCamera.getStdDevFactor();
            this.robotToCamera = titanCamera.getRobotToCameraTransform();
        }

        @Override
        public void updateInputs(final VisionIO.VisionIOInputs inputs) {
            inputs.name = cameraName;
            inputs.stdDevFactor = stdDevFactor;
            inputs.robotToCamera = robotToCamera;
            inputs.latestResult = photonCamera.getLatestResult();
        }
    }

    public static class VisionIONoteTrackingReal implements VisionIO {
        private final PhotonCamera photonCamera;
        private final String cameraName;

        private final Transform3d robotToCamera;

        public VisionIONoteTrackingReal(final TitanCamera titanCamera) {
            this.photonCamera = titanCamera.getPhotonCamera();
            this.cameraName = photonCamera.getName();
            this.robotToCamera = titanCamera.getRobotToCameraTransform();
        }

        @Override
        public void updateInputs(VisionIOInputs inputs) {
            inputs.name = cameraName;
            inputs.stdDevFactor = -1;
            inputs.robotToCamera = robotToCamera;
            inputs.latestResult = photonCamera.getLatestResult();
        }
    }

    private final Map<VisionIOApriltagReal, VisionIO.VisionIOInputs> apriltagVisionIOInputsMap;
    private final Map<VisionIONoteTrackingReal, VisionIO.VisionIOInputs> noteTrackingVisionIOInputsMap;
    private final Map<VisionIOApriltagReal, PhotonPoseEstimator> photonPoseEstimatorMap;

    private final Map<VisionIO, EstimatedRobotPose> estimatedRobotPoseMap;
    private final Map<VisionIO, NoteTrackingResult> noteTrackingResultMap;

    public RealVisionRunner(
            final AprilTagFieldLayout aprilTagFieldLayout,
            final Map<VisionIOApriltagReal, VisionIO.VisionIOInputs> apriltagVisionIOInputsMap,
            final Map<VisionIONoteTrackingReal, VisionIO.VisionIOInputs> noteTrackingVisionIOInputsMap
    ) {
        this.apriltagVisionIOInputsMap = apriltagVisionIOInputsMap;
        this.noteTrackingVisionIOInputsMap = noteTrackingVisionIOInputsMap;

        final Map<VisionIOApriltagReal, PhotonPoseEstimator> poseEstimatorMap = new HashMap<>();
        for (final VisionIOApriltagReal visionIOApriltagReal : apriltagVisionIOInputsMap.keySet()) {
            final PhotonPoseEstimator photonPoseEstimator = new PhotonPoseEstimator(
                    aprilTagFieldLayout,
                    Constants.Vision.MULTI_TAG_POSE_STRATEGY,
                    visionIOApriltagReal.photonCamera,
                    visionIOApriltagReal.titanCamera.getRobotToCameraTransform()
            );
            photonPoseEstimator.setMultiTagFallbackStrategy(PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY);

            poseEstimatorMap.put(visionIOApriltagReal, photonPoseEstimator);
        }

        this.photonPoseEstimatorMap = poseEstimatorMap;
        this.estimatedRobotPoseMap = new HashMap<>();
        this.noteTrackingResultMap = new HashMap<>();
    }

    @SuppressWarnings("DuplicatedCode")
    @Override
    public void periodic() {
        if (ToClose.hasClosed()) {
            return;
        }

        for (
                final Map.Entry<VisionIOApriltagReal, VisionIO.VisionIOInputs>
                        visionIOInputsEntry : apriltagVisionIOInputsMap.entrySet()
        ) {
            final VisionIOApriltagReal visionIO = visionIOInputsEntry.getKey();
            final VisionIO.VisionIOInputs inputs = visionIOInputsEntry.getValue();

            visionIO.periodic();
            visionIO.updateInputs(inputs);

            Logger.processInputs(
                    String.format("%s/%s", PhotonVision.PhotonLogKey, inputs.name),
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

        for (
                final Map.Entry<VisionIONoteTrackingReal, VisionIO.VisionIOInputs>
                        visionIOInputsEntry : noteTrackingVisionIOInputsMap.entrySet()
        ) {
            final VisionIONoteTrackingReal visionIO = visionIOInputsEntry.getKey();
            final VisionIO.VisionIOInputs inputs = visionIOInputsEntry.getValue();

            visionIO.periodic();
            visionIO.updateInputs(inputs);

            Logger.processInputs(
                    String.format("%s/%s", PhotonVision.PhotonLogKey, inputs.name),
                    inputs
            );

            final PhotonPipelineResult pipelineResult = inputs.latestResult;
            VisionUtils.correctPipelineResultTimestamp(pipelineResult);

            Logger.recordOutput(
                    String.format("%s/%s/HasTarget", PhotonVision.PhotonLogKey, inputs.name),
                    pipelineResult.hasTargets()
            );

            final NoteTrackingResult noteTrackingResult = new NoteTrackingResult(inputs.robotToCamera, pipelineResult);
            noteTrackingResultMap.put(visionIO, noteTrackingResult);
        }
    }

    /**
     * Reset the simulated robot {@link Pose3d}.
     * @param robotPose the new robot {@link Pose3d}
     */
    @Override
    public void resetRobotPose(final Pose3d robotPose) {}

    @Override
    public Map<VisionIOApriltagReal, VisionIO.VisionIOInputs> getApriltagVisionIOInputsMap() {
        return apriltagVisionIOInputsMap;
    }

    @Override
    public Map<VisionIONoteTrackingReal, VisionIO.VisionIOInputs> getNoteTrackingVisionIOInputsMap() {
        return noteTrackingVisionIOInputsMap;
    }

    @Override
    public EstimatedRobotPose getEstimatedRobotPose(final VisionIO visionIO) {
        return estimatedRobotPoseMap.get(visionIO);
    }

    @Override
    public NoteTrackingResult getNoteTrackingResult(final VisionIO visionIO) {
        return noteTrackingResultMap.get(visionIO);
    }
}
