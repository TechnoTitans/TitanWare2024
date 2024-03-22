package frc.robot.subsystems.vision;

import frc.robot.constants.Constants;
import frc.robot.utils.logging.LogUtils;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;
import org.photonvision.EstimatedRobotPose;

public interface PhotonVisionIO {
    class PhotonVisionIOInputs implements LoggableInputs {
        public String name = "";
        public double stdDevFactor = 1.0;
        public double[] pipelineResultTargets = new double[] {};
        public String primaryStrategy = Constants.Vision.MULTI_TAG_POSE_STRATEGY.toString();
        public EstimatedRobotPose estimatedRobotPose;
        public EstimatedRobotPose stableEstimatedRobotPose;

        @Override
        public void toLog(final LogTable table) {
            table.put("Name", name);
            table.put("StdDevFactor", stdDevFactor);
            table.put("PipelineResultTargets", pipelineResultTargets);
            table.put("PrimaryStrategy", primaryStrategy);

            LogUtils.serializePhotonVisionEstimatedRobotPose(
                    table, "EstimatedRobotPose", estimatedRobotPose
            );
            LogUtils.serializePhotonVisionEstimatedRobotPose(
                    table, "StableEstimatedRobotPose", stableEstimatedRobotPose
            );
        }

        @Override
        public void fromLog(LogTable table) {
            this.name = table.get("Name", "unknown");
            this.stdDevFactor = table.get("StdDevFactor", Constants.Vision.VISION_CAMERA_DEFAULT_STD_DEV_FACTOR);
            this.pipelineResultTargets = table.get("PipelineResultTargets", new double[] {});
            this.primaryStrategy = table.get(
                    "PrimaryStrategy", Constants.Vision.MULTI_TAG_POSE_STRATEGY.toString()
            );

            this.estimatedRobotPose =
                    LogUtils.deserializePhotonVisionEstimatedRobotPose(table, "EstimatedRobotPose");
            this.stableEstimatedRobotPose =
                    LogUtils.deserializePhotonVisionEstimatedRobotPose(table, "StableEstimatedRobotPose");
        }
    }

    /**
     * Updates the set of loggable inputs.
     * @param inputs Logged class of IOInputs
     * @see PhotonVisionIOInputs
     */
    default void updateInputs(final PhotonVisionIOInputs inputs) {}

    default void periodic() {}
}
