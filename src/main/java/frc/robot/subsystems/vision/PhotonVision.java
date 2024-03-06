package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.constants.Constants;
import frc.robot.subsystems.drive.Swerve;
import frc.robot.utils.PoseUtils;
import frc.robot.utils.gyro.GyroUtils;
import frc.robot.utils.subsystems.VirtualSubsystem;
import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.io.UncheckedIOException;
import java.util.*;
import java.util.stream.Collectors;

public class PhotonVision extends VirtualSubsystem {
    public static final String photonLogKey = "PhotonVision";

    public static final double TRANSLATION_VELOCITY_TOLERANCE = 0.1;
    public static final double ANGULAR_VELOCITY_TOLERANCE = 0.1;

    public static final AprilTagFieldLayout apriltagFieldLayout;

    static {
        AprilTagFieldLayout layout;
        try {
            layout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
        } catch (final UncheckedIOException uncheckedIOException) {
            layout = null;
            DriverStation.reportError("Failed to load AprilTagFieldLayout", uncheckedIOException.getStackTrace());
        }

        apriltagFieldLayout = layout;

        if (apriltagFieldLayout != null) {
            apriltagFieldLayout.setOrigin(AprilTagFieldLayout.OriginPosition.kBlueAllianceWallRightSide);
        }
    }

    private static PhotonVisionApriltagsReal.IOApriltagsReal makePhotonVisionRealIO(final TitanCamera titanCamera) {
        return new PhotonVisionApriltagsReal.IOApriltagsReal(titanCamera, PhotonVision.apriltagFieldLayout);
    }

    private static PhotonVisionApriltagsSim.IOApriltagsSim makePhotonVisionSimIO(
            final TitanCamera titanCamera,
            final VisionSystemSim visionSystemSim
    ) {
        return new PhotonVisionApriltagsSim.IOApriltagsSim(
                titanCamera, PhotonVision.apriltagFieldLayout, visionSystemSim
        );
    }

    @SafeVarargs
    public static <T extends PhotonVisionIO> Map<T, PhotonVisionIO.PhotonVisionIOInputs> makePhotonVisionIOInputsMap(
            final T... photonVisionIOs
    ) {
        return Arrays.stream(photonVisionIOs).collect(Collectors.toMap(
                photonVisionIO -> photonVisionIO,
                photonVisionIO -> new PhotonVisionIO.PhotonVisionIOInputs()
        ));
    }

    private final PhotonVisionRunner<? extends PhotonVisionIO> runner;
    private final Map<? extends PhotonVisionIO, PhotonVisionIO.PhotonVisionIOInputs> photonVisionIOInputsMap;

    private final Swerve swerve;
    private final SwerveDrivePoseEstimator poseEstimator;
    private final Map<PhotonVisionIO, EstimatedRobotPose> lastStableEstimatedPosesByCamera;

    public PhotonVision(
            final Constants.RobotMode robotMode,
            final Swerve swerve,
            final SwerveDrivePoseEstimator poseEstimator
    ) {
        this.runner = switch (robotMode) {
            case REAL -> new PhotonVisionApriltagsReal(
                    PhotonVision.makePhotonVisionIOInputsMap(
                            makePhotonVisionRealIO(TitanCamera.PHOTON_FL_APRILTAG),
                            makePhotonVisionRealIO(TitanCamera.PHOTON_FR_APRILTAG)
                    )
            );
            case SIM -> {
                final VisionSystemSim visionSystemSim = new VisionSystemSim(PhotonVision.photonLogKey);
                yield new PhotonVisionApriltagsSim(
                        swerve,
                        new SwerveDriveOdometry(
                                swerve.getKinematics(),
                                swerve.getYaw(),
                                swerve.getModulePositions(),
                                new Pose2d()
                        ),
                        PhotonVision.apriltagFieldLayout,
                        visionSystemSim,
                        PhotonVision.makePhotonVisionIOInputsMap(
                                makePhotonVisionSimIO(TitanCamera.PHOTON_FL_APRILTAG, visionSystemSim),
                                makePhotonVisionSimIO(TitanCamera.PHOTON_FR_APRILTAG, visionSystemSim)
                        )
                );
            }
            case REPLAY -> new PhotonVisionRunner<>() {};
        };

        this.swerve = swerve;
        this.poseEstimator = poseEstimator;
        this.photonVisionIOInputsMap = runner.getPhotonVisionIOInputsMap();

        this.lastStableEstimatedPosesByCamera = new HashMap<>();

        final Pose2d estimatedPose = poseEstimator.getEstimatedPosition();
        resetPosition(estimatedPose);
    }

    public enum EstimationRejectionReason {
        DID_NOT_REJECT(0),
        ESTIMATED_POSE_OBJECT_NULL(1),
        ESTIMATED_POSE_OR_TIMESTAMP_OR_TARGETS_INVALID(2),
        POSE_NOT_IN_FIELD(3),
        LAST_ESTIMATED_POSE_TIMESTAMP_INVALID_OR_TOO_CLOSE(4),
        POSE_IMPOSSIBLE_VELOCITY(5);

        private final int id;
        EstimationRejectionReason(final int id) {
            this.id = id;
        }

        public int getId() {
            return id;
        }

        public static boolean wasRejected(final EstimationRejectionReason rejectionReason) {
            return rejectionReason != DID_NOT_REJECT;
        }

        public boolean wasRejected() {
            return wasRejected(this);
        }
    }

    public EstimationRejectionReason shouldRejectEstimation(
            final EstimatedRobotPose lastEstimatedRobotPose,
            final EstimatedRobotPose estimatedRobotPose
    ) {
        if (estimatedRobotPose == null) {
            // reject immediately if the estimated pose itself is null
            return EstimationRejectionReason.ESTIMATED_POSE_OBJECT_NULL;
        }

        if (estimatedRobotPose.estimatedPose == null
                || estimatedRobotPose.timestampSeconds == -1
                || estimatedRobotPose.targetsUsed.isEmpty()) {
            // reject immediately if null estimatedPose, timestamp is invalid, or no targets used
            return EstimationRejectionReason.ESTIMATED_POSE_OR_TIMESTAMP_OR_TARGETS_INVALID;
        }

        if (lastEstimatedRobotPose == null) {
            // do not reject if there was no last estimation at all (this is different from an invalid last estimation)
            // likely, this is the first time we have an estimation, make sure we accept this estimation
            return EstimationRejectionReason.DID_NOT_REJECT;
        }

        final Pose3d nextEstimatedPosition = estimatedRobotPose.estimatedPose;

        if (!PoseUtils.isInField(nextEstimatedPosition)) {
//             reject if pose not within the field
            return EstimationRejectionReason.POSE_NOT_IN_FIELD;
        }

        final double secondsSinceLastUpdate =
                estimatedRobotPose.timestampSeconds - lastEstimatedRobotPose.timestampSeconds;

        if (lastEstimatedRobotPose.timestampSeconds == -1 || secondsSinceLastUpdate <= 0) {
            // TODO: do we always need to reject immediately here? maybe we can still use the next estimation even
            //  if the last estimation had no timestamp or was very close
            return EstimationRejectionReason.LAST_ESTIMATED_POSE_TIMESTAMP_INVALID_OR_TOO_CLOSE;
        }

        final Pose2d nextEstimatedPosition2d = nextEstimatedPosition.toPose2d();
        final Pose2d lastEstimatedPosition2d = lastEstimatedRobotPose.estimatedPose.toPose2d();
        final Twist2d twist2dToNewEstimation = lastEstimatedPosition2d.log(nextEstimatedPosition2d);

        final double xVel = twist2dToNewEstimation.dx / secondsSinceLastUpdate;
        final double yVel = twist2dToNewEstimation.dy / secondsSinceLastUpdate;
        final double thetaVel = twist2dToNewEstimation.dtheta / secondsSinceLastUpdate;
        final double translationVel = Math.hypot(xVel, yVel);

        Logger.recordOutput(
                photonLogKey + "/TranslationVel", translationVel
        );
        Logger.recordOutput(
                photonLogKey + "/ThetaVel", thetaVel
        );

        // assume hypot is positive (>= 0)
        if ((translationVel >= Constants.Swerve.ROBOT_MAX_SPEED_MPS + PhotonVision.TRANSLATION_VELOCITY_TOLERANCE)
                || (Math.abs(thetaVel) >= Constants.Swerve.ROBOT_MAX_ANGULAR_SPEED_RAD_PER_SEC
                    + PhotonVision.ANGULAR_VELOCITY_TOLERANCE)) {
            // reject sudden pose changes resulting in an impossible velocity (cannot reach)
            return EstimationRejectionReason.POSE_IMPOSSIBLE_VELOCITY;
        }

        // TODO: more rejection stuff
        return EstimationRejectionReason.DID_NOT_REJECT;
    }

    private void update() {
        for (
                final Map.Entry<? extends PhotonVisionIO, PhotonVisionIO.PhotonVisionIOInputs>
                        photonVisionIOInputsEntry : photonVisionIOInputsMap.entrySet()
        ) {

            final PhotonVisionIO photonVisionIO = photonVisionIOInputsEntry.getKey();
            final PhotonVisionIO.PhotonVisionIOInputs photonVisionIOInputs = photonVisionIOInputsEntry.getValue();

            final EstimatedRobotPose stableEstimatedRobotPose = photonVisionIOInputs.stableEstimatedRobotPose;
            final EstimatedRobotPose estimatedRobotPose = photonVisionIOInputs.estimatedRobotPose;

            if (estimatedRobotPose == null) {
                // skip the trouble of calling/indexing things if the estimatedRobotPose is null
                continue;
            }

            final EstimatedRobotPose lastSavedEstimatedPose = lastStableEstimatedPosesByCamera.get(photonVisionIO);
            final EstimationRejectionReason rejectionReason =
                    shouldRejectEstimation(lastSavedEstimatedPose, estimatedRobotPose);

            Logger.recordOutput(
                    photonLogKey + "/RejectionReason", rejectionReason.getId()
            );

            if (rejectionReason.wasRejected()) {
                continue;
            }

            lastStableEstimatedPosesByCamera.put(photonVisionIO, stableEstimatedRobotPose);

            // TODO: get better calibrations on cameras or get better cameras
            poseEstimator.addVisionMeasurement(
                    estimatedRobotPose.estimatedPose.toPose2d(),
                    estimatedRobotPose.timestampSeconds
            );
        }
    }

    public void logVisionData() {
        final Set<Integer> apriltagIds = new HashSet<>();
        final List<Pose3d> estimatedPoses = new ArrayList<>();
        final List<Pose3d> apriltagPoses = new ArrayList<>();

        for (final EstimatedRobotPose estimatedRobotPose : lastStableEstimatedPosesByCamera.values()) {
            if (estimatedRobotPose == null) {
                continue;
            }

            final Set<Integer> ids = estimatedRobotPose.targetsUsed.stream()
                    .map(PhotonTrackedTarget::getFiducialId)
                    .collect(Collectors.toSet());

            estimatedPoses.add(estimatedRobotPose.estimatedPose);
            apriltagIds.addAll(ids);
            apriltagPoses.addAll(
                    ids.stream().map(
                            id -> apriltagFieldLayout.getTagPose(id).orElse(new Pose3d())
                    ).toList()
            );
        }

        Logger.recordOutput(
                PhotonVision.photonLogKey + "/EstimatedRobotPose3dsByCamera",
                estimatedPoses.toArray(Pose3d[]::new)
        );

        Logger.recordOutput(
                PhotonVision.photonLogKey + "/EstimatedRobotPose2dsByCamera",
                estimatedPoses.stream().map(Pose3d::toPose2d).toArray(Pose2d[]::new)
        );

        Logger.recordOutput(
                PhotonVision.photonLogKey + "/ApriltagIds",
                apriltagIds.stream().mapToLong(Number::longValue).toArray()
        );

        Logger.recordOutput(
                PhotonVision.photonLogKey + "/ApriltagPose3ds",
                apriltagPoses.toArray(Pose3d[]::new)
        );

        Logger.recordOutput(
                PhotonVision.photonLogKey + "/ApriltagPose2ds",
                apriltagPoses.stream().map(Pose3d::toPose2d).toArray(Pose2d[]::new)
        );
    }

    @Override
    public void periodic() {
        runner.periodic();

        // Update and log PhotonVision results
        update();
        logVisionData();
    }

    public void resetPosition(final Pose2d robotPose, final Rotation2d robotYaw) {
        poseEstimator.resetPosition(robotYaw, swerve.getModulePositions(), robotPose);
        runner.resetRobotPose(GyroUtils.robotPose2dToPose3dWithGyro(
                new Pose2d(robotPose.getTranslation(), robotYaw),
                GyroUtils.rpyToRotation3d(swerve.getRoll(), swerve.getPitch(), swerve.getYaw())
        ));
    }

    public void resetPosition(final Pose2d robotPose) {
        resetPosition(robotPose, swerve.getYaw());
    }
}
