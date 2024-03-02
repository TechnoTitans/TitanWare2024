package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.constants.FieldConstants;

public class PoseUtils {
    private PoseUtils() {}

    public static boolean isInField(final Pose3d pose3d) {
        return pose3d.getX() >= -FieldConstants.FIELD_WITHIN_BORDER_MARGIN
                && pose3d.getX() <= FieldConstants.FIELD_LENGTH_X_METERS + FieldConstants.FIELD_WITHIN_BORDER_MARGIN
                && pose3d.getY() >= -FieldConstants.FIELD_WITHIN_BORDER_MARGIN
                && pose3d.getY() <= FieldConstants.FIELD_WIDTH_Y_METERS + FieldConstants.FIELD_WITHIN_BORDER_MARGIN
                && pose3d.getZ() >= -FieldConstants.FIELD_WITHIN_Z_MARGIN
                && pose3d.getZ() <= FieldConstants.FIELD_WITHIN_Z_MARGIN;
    }
}
