package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;

import java.util.Map;

public interface PhotonVisionRunner<T extends PhotonVisionIO> {
    default void periodic() {}
    default void resetRobotPose(final Pose3d pose3d) {}
    default Map<T, PhotonVisionIO.PhotonVisionIOInputs> getPhotonVisionIOInputsMap() {
        return Map.of();
    }
}
