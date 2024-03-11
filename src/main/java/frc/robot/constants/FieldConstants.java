package frc.robot.constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.utils.PoseUtils;

public class FieldConstants {
    public static final double FIELD_LENGTH_X_METERS = Units.inchesToMeters(651.223);
    public static final double FIELD_WIDTH_Y_METERS = Units.inchesToMeters(323.277);

    public static final Pose2d BLUE_SPEAKER_POSE =
            new Pose2d(new Translation2d(-0.0381, 5.547868), Rotation2d.fromDegrees(0));
    public static final Pose2d RED_SPEAKER_POSE =
            new Pose2d(new Translation2d(16.579342, 5.547868), Rotation2d.fromDegrees(180));

    public static final Pose2d BLUE_AMP_POSE =
            new Pose2d(
                    Units.inchesToMeters(72.455),
                    FIELD_WIDTH_Y_METERS - Units.inchesToMeters(20),
                    Rotation2d.fromRadians(-Math.PI / 2)
            );

    public static final Pose2d RED_AMP_POSE = PoseUtils.flip(BLUE_AMP_POSE);

    public static Pose2d getAmpScoringPose() {
        return DriverStation.getAlliance()
                .filter(value -> value == DriverStation.Alliance.Red)
                .map(value -> RED_AMP_POSE).orElse(BLUE_AMP_POSE);
    }

    public static Pose2d getSpeakerPose() {
        return DriverStation.getAlliance()
                .filter(value -> value == DriverStation.Alliance.Red)
                .map(value -> RED_SPEAKER_POSE).orElse(BLUE_SPEAKER_POSE);
    }
}
