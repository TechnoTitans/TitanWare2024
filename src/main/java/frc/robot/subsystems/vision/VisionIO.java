package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.constants.Constants;
import frc.robot.utils.logging.LogUtils;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;
import org.photonvision.targeting.PhotonPipelineResult;

public interface VisionIO {
    class VisionIOInputs implements LoggableInputs {
        public String name = "";
        public double stdDevFactor = 1.0;
        public Transform3d robotToCamera;
        public PhotonPipelineResult latestResult;

        @Override
        public void toLog(final LogTable table) {
            table.put("Name", name);
            table.put("StdDevFactor", stdDevFactor);
            table.put("RobotToCamera", robotToCamera);
            LogUtils.serializePhotonPipelineResult(table, "LatestResult", latestResult);
        }

        @Override
        public void fromLog(final LogTable table) {
            this.name = table.get("Name", "unknown");
            this.stdDevFactor = table.get("StdDevFactor", Constants.Vision.VISION_CAMERA_DEFAULT_STD_DEV_FACTOR);
            this.robotToCamera = table.get("RobotToCamera", new Transform3d());
            this.latestResult = LogUtils.deserializePhotonPipelineResult(table, "LatestResult");
        }
    }

    /**
     * Updates the set of loggable inputs.
     * @param inputs Logged class of IOInputs
     * @see VisionIOInputs
     */
    default void updateInputs(final VisionIOInputs inputs) {}

    default void periodic() {}
}
