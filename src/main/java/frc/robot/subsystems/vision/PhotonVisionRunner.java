package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.subsystems.vision.result.NoteTrackingResult;
import org.photonvision.EstimatedRobotPose;

import java.util.Map;

public interface PhotonVisionRunner {
    default void periodic() {}
    default void resetRobotPose(final Pose3d pose3d) {}
    default Map<? extends VisionIO, VisionIO.VisionIOInputs> getApriltagVisionIOInputsMap() {
        return Map.of();
    }

    default EstimatedRobotPose getEstimatedRobotPose(final VisionIO visionIO) {
        return null;
    }

    default NoteTrackingResult getNoteTrackingResult(final VisionIO visionIO) {
        return null;
    }
}
