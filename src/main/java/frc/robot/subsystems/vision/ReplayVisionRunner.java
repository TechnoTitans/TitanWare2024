package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose3d;
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

public class ReplayVisionRunner implements PhotonVisionRunner {
    public static class VisionIOReplay implements VisionIO {
        private final TitanCamera titanCamera;
        private final PhotonCamera photonCamera;

        public VisionIOReplay(final TitanCamera titanCamera) {
            this.titanCamera = titanCamera;
            this.photonCamera = titanCamera.getPhotonCamera();
        }
    }

    private final Map<ReplayVisionRunner.VisionIOReplay, String> visionIONames;
    private final Map<ReplayVisionRunner.VisionIOReplay, VisionIO.VisionIOInputs> apriltagVisionIOInputsMap;
    private final Map<ReplayVisionRunner.VisionIOReplay, VisionIO.VisionIOInputs> noteTrackingVisionIOInputsMap;
    private final Map<ReplayVisionRunner.VisionIOReplay, PhotonPoseEstimator> photonPoseEstimatorMap;

    private final Map<VisionIO, EstimatedRobotPose> estimatedRobotPoseMap;
    private final Map<VisionIO, NoteTrackingResult> noteTrackingResultMap;

    public ReplayVisionRunner(
            final AprilTagFieldLayout aprilTagFieldLayout,
            final Map<ReplayVisionRunner.VisionIOReplay, VisionIO.VisionIOInputs> apriltagVisionIOInputsMap,
            final Map<ReplayVisionRunner.VisionIOReplay, VisionIO.VisionIOInputs> noteTrackingVisionIOInputsMap
    ) {
        this.apriltagVisionIOInputsMap = apriltagVisionIOInputsMap;
        this.noteTrackingVisionIOInputsMap = noteTrackingVisionIOInputsMap;

        final Map<ReplayVisionRunner.VisionIOReplay, String> visionIONames = new HashMap<>();
        final Map<ReplayVisionRunner.VisionIOReplay, PhotonPoseEstimator> poseEstimatorMap = new HashMap<>();
        for (final ReplayVisionRunner.VisionIOReplay visionIOApriltagsReplay : apriltagVisionIOInputsMap.keySet()) {
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

        for (final ReplayVisionRunner.VisionIOReplay visionIONoteTrackReplay : noteTrackingVisionIOInputsMap.keySet()) {
            visionIONames.put(visionIONoteTrackReplay, visionIONoteTrackReplay.photonCamera.getName());
        }

        this.visionIONames = visionIONames;
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
                final Map.Entry<ReplayVisionRunner.VisionIOReplay, VisionIO.VisionIOInputs>
                        photonVisionIOInputsEntry : apriltagVisionIOInputsMap.entrySet()
        ) {
            final ReplayVisionRunner.VisionIOReplay visionIO = photonVisionIOInputsEntry.getKey();
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

        for (
                final Map.Entry<ReplayVisionRunner.VisionIOReplay, VisionIO.VisionIOInputs>
                        photonVisionIOInputsEntry : noteTrackingVisionIOInputsMap.entrySet()
        ) {
            final ReplayVisionRunner.VisionIOReplay visionIO = photonVisionIOInputsEntry.getKey();
            final VisionIO.VisionIOInputs inputs = photonVisionIOInputsEntry.getValue();

            visionIO.periodic();
            visionIO.updateInputs(inputs);

            Logger.processInputs(
                    String.format("%s/%s", PhotonVision.PhotonLogKey, visionIONames.get(visionIO)),
                    inputs
            );

            final PhotonPipelineResult pipelineResult = inputs.latestResult;
            VisionUtils.correctPipelineResultTimestamp(pipelineResult);

            Logger.recordOutput(
                    String.format("%s/%s/HasTarget", PhotonVision.PhotonLogKey, visionIONames.get(visionIO)),
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
    public Map<ReplayVisionRunner.VisionIOReplay, VisionIO.VisionIOInputs> getApriltagVisionIOInputsMap() {
        return apriltagVisionIOInputsMap;
    }

    @Override
    public Map<ReplayVisionRunner.VisionIOReplay, VisionIO.VisionIOInputs> getNoteTrackingVisionIOInputsMap() {
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
