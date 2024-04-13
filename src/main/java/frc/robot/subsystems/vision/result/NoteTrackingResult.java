package frc.robot.subsystems.vision.result;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.vision.RealVisionRunner;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.Optional;
import java.util.function.Function;

public class NoteTrackingResult {
    private static final double DistanceOffsetMeters = Units.inchesToMeters(0);

    private final Transform3d robotToCamera;

    public final PhotonPipelineResult pipelineResult;
    public final boolean hasTargets;

    public final PhotonTrackedTarget bestTarget;
    public final Rotation2d bestTargetYaw;
    public final Rotation2d bestTargetPitch;
    public final double bestTargetDistance;

    public NoteTrackingResult(final Transform3d robotToCamera, final PhotonPipelineResult result) {
        this.robotToCamera = robotToCamera;
        this.pipelineResult = result;
        if (result == null || !result.hasTargets()) {
            this.hasTargets = false;
            this.bestTarget = null;
            this.bestTargetYaw = new Rotation2d();
            this.bestTargetPitch = new Rotation2d();
            this.bestTargetDistance = 0;
        } else {
            this.hasTargets = true;
            this.bestTarget = pipelineResult.getBestTarget();
            this.bestTargetYaw = getTargetYaw(bestTarget);
            this.bestTargetPitch = getTargetPitch(bestTarget);
            this.bestTargetDistance = getNoteDistance(robotToCamera, bestTarget);
        }
    }

    public Optional<Pose2d> getBestNotePose(final Function<Double, Optional<Pose2d>> timestampedRobotPoseFunction) {
        if (!hasTargets) {
            return Optional.empty();
        }

        final Optional<Pose2d> optionalRobotPose = timestampedRobotPoseFunction.apply(pipelineResult.getTimestampSeconds());
        return optionalRobotPose.map(pose2d -> getNotePose(pose2d, robotToCamera, bestTarget));
    }

    public Pose2d[] getNotePoses(final Function<Double, Optional<Pose2d>> timestampedRobotPoseFunction) {
        if (!hasTargets) {
            return new Pose2d[]{};
        }

        final Optional<Pose2d> optionalRobotPose = timestampedRobotPoseFunction.apply(pipelineResult.getTimestampSeconds());
        if (optionalRobotPose.isEmpty()) {
            return new Pose2d[]{};
        }

        final int nTargets = pipelineResult.targets.size();
        final Pose2d robotPose = optionalRobotPose.get();
        final Pose2d[] notePoses = new Pose2d[nTargets];
        for (int i = 0; i < nTargets; i++) {
            notePoses[i] = getNotePose(robotPose, robotToCamera, pipelineResult.targets.get(i));
        }

        return notePoses;
    }

    public static Rotation2d getTargetYaw(final PhotonTrackedTarget trackedTarget) {
        return Rotation2d.fromDegrees(-trackedTarget.getYaw());
    }

    public static Rotation2d getTargetPitch(final PhotonTrackedTarget trackedTarget) {
        return Rotation2d.fromDegrees(-trackedTarget.getPitch());
    }

    public static double getNoteDistance(final Transform3d robotToCamera, final PhotonTrackedTarget trackedTarget) {
        return PhotonUtils.calculateDistanceToTargetMeters(
                robotToCamera.getZ(),
                RealVisionRunner.VisionIONoteTrackingReal.NOTE_HEIGHT_Z,
                -robotToCamera.getRotation().getY(), // CCW+ convert to CW+
                Units.degreesToRadians(trackedTarget.getPitch()) // doesn't need negative, PhotonUtils expects CW+
        ) + DistanceOffsetMeters;
    }

    public static Pose2d getNotePose(
            final Pose2d poseAtTimestamp,
            final Transform3d robotToCamera,
            final PhotonTrackedTarget trackedTarget
    ) {
        final Transform2d robotToNote = new Transform2d(
                new Translation2d(getNoteDistance(robotToCamera, trackedTarget), getTargetYaw(trackedTarget)),
                getTargetYaw(trackedTarget)
        );

        return poseAtTimestamp.transformBy(robotToNote);
    }
}
