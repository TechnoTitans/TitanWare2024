package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import frc.robot.constants.Constants;
import frc.robot.constants.SimConstants;
import frc.robot.subsystems.drive.Swerve;
import frc.robot.subsystems.vision.cameras.TitanCamera;
import frc.robot.subsystems.vision.result.NoteTrackingResult;
import frc.robot.utils.PoseUtils;
import frc.robot.utils.closeables.ToClose;
import frc.robot.utils.gyro.GyroUtils;
import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.simulation.VisionTargetSim;
import org.photonvision.targeting.PhotonPipelineResult;

import java.util.HashMap;
import java.util.Map;

public class SimVisionRunner implements PhotonVisionRunner {
    public static class VisionIOApriltagsSim implements VisionIO {
        public final TitanCamera titanCamera;
        public final PhotonCamera photonCamera;
        public final String cameraName;

        public final double stdDevFactor;
        public final Transform3d robotToCamera;

        public VisionIOApriltagsSim(
                final TitanCamera titanCamera,
                final VisionSystemSim visionSystemSim
        ) {
            this.titanCamera = titanCamera;
            this.photonCamera = titanCamera.getPhotonCamera();
            this.cameraName = photonCamera.getName();

            this.stdDevFactor = titanCamera.getStdDevFactor();
            this.robotToCamera = titanCamera.getRobotToCameraTransform();

            final PhotonCameraSim photonCameraSim =
                    new PhotonCameraSim(titanCamera.getPhotonCamera(), titanCamera.toSimCameraProperties());

            photonCameraSim.enableDrawWireframe(true);
            photonCameraSim.enableRawStream(true);
            photonCameraSim.enableProcessedStream(true);

            ToClose.add(photonCameraSim);
            visionSystemSim.addCamera(photonCameraSim, titanCamera.getRobotToCameraTransform());
        }

        @Override
        public void updateInputs(final VisionIO.VisionIOInputs inputs) {
            inputs.name = cameraName;
            inputs.stdDevFactor = stdDevFactor;
            inputs.robotToCamera = robotToCamera;
            inputs.latestResult = photonCamera.getLatestResult();
        }
    }

    public static class VisionIONoteTrackingSim implements VisionIO {
        public final TitanCamera titanCamera;
        public final PhotonCamera photonCamera;
        public final String cameraName;

        public final Transform3d robotToCamera;

        public VisionIONoteTrackingSim(
                final TitanCamera titanCamera,
                final VisionSystemSim visionSystemSim
        ) {
            this.titanCamera = titanCamera;
            this.photonCamera = titanCamera.getPhotonCamera();
            this.cameraName = photonCamera.getName();

            this.robotToCamera = titanCamera.getRobotToCameraTransform();

            final PhotonCameraSim photonCameraSim =
                    new PhotonCameraSim(titanCamera.getPhotonCamera(), titanCamera.toSimCameraProperties());

            photonCameraSim.enableDrawWireframe(true);
            photonCameraSim.enableRawStream(true);
            photonCameraSim.enableProcessedStream(true);

            ToClose.add(photonCameraSim);
            visionSystemSim.addCamera(photonCameraSim, titanCamera.getRobotToCameraTransform());
        }

        @Override
        public void updateInputs(final VisionIOInputs inputs) {
            inputs.name = cameraName;
            inputs.stdDevFactor = -1;
            inputs.robotToCamera = robotToCamera;
            inputs.latestResult = photonCamera.getLatestResult();
        }
    }

    private final Swerve swerve;
    private final SwerveDriveOdometry visionIndependentOdometry;
    private final VisionSystemSim visionSystemSim;

    private final Map<VisionIOApriltagsSim, VisionIO.VisionIOInputs> apriltagVisionIOInputsMap;
    private final Map<VisionIONoteTrackingSim, VisionIO.VisionIOInputs> noteTrackingVisionIOInputsMap;
    private final Map<VisionIOApriltagsSim, PhotonPoseEstimator> photonPoseEstimatorMap;

    private final Map<VisionIO, EstimatedRobotPose> estimatedRobotPoseMap;
    private final Map<VisionIO, NoteTrackingResult> noteTrackingResultMap;

    public SimVisionRunner(
            final Swerve swerve,
            final SwerveDriveOdometry visionIndependentOdometry,
            final AprilTagFieldLayout aprilTagFieldLayout,
            final VisionSystemSim visionSystemSim,
            final Pose2d[] simNotePoses,
            final Map<VisionIOApriltagsSim, VisionIO.VisionIOInputs> apriltagVisionIOInputsMap,
            final Map<VisionIONoteTrackingSim, VisionIO.VisionIOInputs> noteTrackingVisionIOInputsMap
    ) {
        this.swerve = swerve;
        this.visionIndependentOdometry = visionIndependentOdometry;
        this.visionSystemSim = visionSystemSim;
        this.visionSystemSim.addAprilTags(aprilTagFieldLayout);

        for (final Pose2d simNotePose : simNotePoses) {
            this.visionSystemSim.addVisionTargets("note", new VisionTargetSim(
                    PoseUtils.note2dTo3d(simNotePose), SimConstants.Vision.NOTE_TARGET_MODEL
            ));
        }

        this.apriltagVisionIOInputsMap = apriltagVisionIOInputsMap;
        this.noteTrackingVisionIOInputsMap = noteTrackingVisionIOInputsMap;

        final Map<VisionIOApriltagsSim, PhotonPoseEstimator> poseEstimatorMap = new HashMap<>();
        for (final VisionIOApriltagsSim visionIOApriltagsSim : apriltagVisionIOInputsMap.keySet()) {
            final PhotonPoseEstimator photonPoseEstimator = new PhotonPoseEstimator(
                    aprilTagFieldLayout,
                    Constants.Vision.MULTI_TAG_POSE_STRATEGY,
                    visionIOApriltagsSim.photonCamera,
                    visionIOApriltagsSim.titanCamera.getRobotToCameraTransform()
            );
            photonPoseEstimator.setMultiTagFallbackStrategy(PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY);

            poseEstimatorMap.put(visionIOApriltagsSim, photonPoseEstimator);
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

        final Pose2d visionIndependentPose =
                visionIndependentOdometry.update(swerve.getYaw(), swerve.getModulePositions());

        visionSystemSim.update(
                GyroUtils.robotPose2dToPose3dWithGyro(
                        visionIndependentPose,
                        new Rotation3d(
                                swerve.getRoll().getRadians(),
                                swerve.getPitch().getRadians(),
                                swerve.getYaw().getRadians()
                        )
                )
        );

        for (
                final Map.Entry<VisionIOApriltagsSim, VisionIO.VisionIOInputs>
                        photonVisionIOInputsEntry : apriltagVisionIOInputsMap.entrySet()
        ) {
            final VisionIOApriltagsSim visionIO = photonVisionIOInputsEntry.getKey();
            final VisionIO.VisionIOInputs inputs = photonVisionIOInputsEntry.getValue();

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
                final Map.Entry<VisionIONoteTrackingSim, VisionIO.VisionIOInputs>
                        photonVisionIOInputsEntry : noteTrackingVisionIOInputsMap.entrySet()
        ) {
            final VisionIONoteTrackingSim visionIO = photonVisionIOInputsEntry.getKey();
            final VisionIO.VisionIOInputs inputs = photonVisionIOInputsEntry.getValue();

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
    public void resetRobotPose(final Pose3d robotPose) {
        visionSystemSim.resetRobotPose(robotPose);
        visionIndependentOdometry.resetPosition(
                robotPose.toPose2d().getRotation(), swerve.getModulePositions(), robotPose.toPose2d()
        );
    }

    @Override
    public Map<VisionIOApriltagsSim, VisionIO.VisionIOInputs> getApriltagVisionIOInputsMap() {
        return apriltagVisionIOInputsMap;
    }

    @Override
    public Map<VisionIONoteTrackingSim, VisionIO.VisionIOInputs> getNoteTrackingVisionIOInputsMap() {
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
