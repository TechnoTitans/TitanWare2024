package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import frc.robot.constants.Constants;
import frc.robot.subsystems.drive.Swerve;
import frc.robot.utils.closeables.ToClose;
import frc.robot.utils.gyro.GyroUtils;
import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.List;
import java.util.Map;
import java.util.Optional;

public class PhotonVisionApriltagsSim implements PhotonVisionRunner<PhotonVisionApriltagsSim.IOApriltagsSim> {
    public static class IOApriltagsSim implements PhotonVisionIO {
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

        public IOApriltagsSim(
                final TitanCamera titanCamera,
                final AprilTagFieldLayout blueSideApriltagFieldLayout,
                final VisionSystemSim visionSystemSim
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

            final PhotonCameraSim photonCameraSim =
                    new PhotonCameraSim(titanCamera.getPhotonCamera(), titanCamera.toSimCameraProperties());

            photonCameraSim.enableDrawWireframe(true);
            photonCameraSim.enableRawStream(true);
            photonCameraSim.enableProcessedStream(true);

            ToClose.add(photonCameraSim);
            visionSystemSim.addCamera(photonCameraSim, titanCamera.getRobotRelativeToCameraTransform());
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


    private final Swerve swerve;
    private final SwerveDriveOdometry visionIndependentOdometry;
    private final Map<IOApriltagsSim, PhotonVisionIO.PhotonVisionIOInputs> photonVisionIOInputsMap;

    private final VisionSystemSim visionSystemSim;

    public PhotonVisionApriltagsSim(
            final Swerve swerve,
            final SwerveDriveOdometry visionIndependentOdometry,
            final AprilTagFieldLayout aprilTagFieldLayout,
            final VisionSystemSim visionSystemSim,
            final Map<IOApriltagsSim, PhotonVisionIO.PhotonVisionIOInputs> photonVisionIOInputsMap
    ) {
        this.swerve = swerve;
        this.visionIndependentOdometry = visionIndependentOdometry;
        this.photonVisionIOInputsMap = photonVisionIOInputsMap;

        this.visionSystemSim = visionSystemSim;
        this.visionSystemSim.addAprilTags(aprilTagFieldLayout);
    }

    @Override
    public void periodic() {
        if (ToClose.hasClosed()) {
            // do not try to update if we've already closed or if we cannot continue running
            return;
        }

        for (
                final Map.Entry<IOApriltagsSim, PhotonVisionIO.PhotonVisionIOInputs>
                        photonVisionIOInputsEntry : photonVisionIOInputsMap.entrySet()
        ) {
            final IOApriltagsSim ioApriltagsSim = photonVisionIOInputsEntry.getKey();
            final PhotonVisionIO.PhotonVisionIOInputs ioInputs = photonVisionIOInputsEntry.getValue();

            ioApriltagsSim.periodic();
            ioApriltagsSim.updateInputs(ioInputs);

            Logger.processInputs(
                    String.format("%s/%s", PhotonVision.PhotonLogKey, ioApriltagsSim.cameraName),
                    ioInputs
            );
        }

        final Pose2d visionIndependentPose =
                visionIndependentOdometry.update(swerve.getYaw(), swerve.getModulePositions());

        visionSystemSim.update(
                GyroUtils.robotPose2dToPose3dWithGyro(
                        visionIndependentPose,
                        GyroUtils.rpyToRotation3d(swerve.getRoll(), swerve.getPitch(), swerve.getYaw())
                )
        );
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
    public Map<PhotonVisionApriltagsSim.IOApriltagsSim, PhotonVisionIO.PhotonVisionIOInputs> getPhotonVisionIOInputsMap() {
        return photonVisionIOInputsMap;
    }
}
