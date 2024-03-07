package frc.robot.constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;

public class FieldConstants {
    public static final double FIELD_LENGTH_X_METERS = 16.54175;
    public static final double FIELD_WIDTH_Y_METERS = 8.0137;
    public static final double FIELD_WITHIN_BORDER_MARGIN = 0.5;
    public static final double FIELD_WITHIN_Z_MARGIN = 0.75;

    public static final Pose2d BLUE_SPEAKER_POSE =
            new Pose2d(new Translation2d(-0.0381, 5.547868), Rotation2d.fromDegrees(180));
    public static final Pose2d RED_SPEAKER_POSE =
            new Pose2d(new Translation2d(16.579342, 5.547868), Rotation2d.fromDegrees(0));

    public static Pose2d getSpeakerPose() {
        return DriverStation.getAlliance()
                .filter(value -> value == DriverStation.Alliance.Red)
                .map(value -> RED_SPEAKER_POSE).orElse(BLUE_SPEAKER_POSE);
    }
}
