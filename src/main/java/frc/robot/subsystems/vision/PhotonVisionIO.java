package frc.robot.subsystems.vision;

import frc.robot.constants.Constants;
import frc.robot.utils.logging.LogUtils;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;
import org.photonvision.EstimatedRobotPose;

public interface PhotonVisionIO {
    class PhotonVisionIOInputs implements LoggableInputs {
        public double[] pipelineResultTargets = new double[] {};
        public String primaryStrategy = Constants.Vision.MULTI_TAG_POSE_STRATEGY.toString();
        public EstimatedRobotPose estimatedRobotPose;
        public EstimatedRobotPose stableEstimatedRobotPose;

        @Override
        public void toLog(final LogTable table) {
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
