package frc.robot.subsystems.vision;

import frc.robot.constants.Constants;
import frc.robot.utils.logging.LogUtils;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;
import org.photonvision.targeting.PhotonPipelineResult;

public interface VisionIO {
    class VisionIOInputs implements LoggableInputs {
        public String name = "";
        public double stdDevFactor = 1.0;
        public PhotonPipelineResult latestResult;

        @Override
        public void toLog(final LogTable table) {
            table.put("Name", name);
            table.put("StdDevFactor", stdDevFactor);
            LogUtils.serializePhotonPipelineResult(table, "LatestResult", latestResult);
        }

        @Override
        public void fromLog(final LogTable table) {
            this.name = table.get("Name", "unknown");
            this.stdDevFactor = table.get("StdDevFactor", Constants.Vision.VISION_CAMERA_DEFAULT_STD_DEV_FACTOR);
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
