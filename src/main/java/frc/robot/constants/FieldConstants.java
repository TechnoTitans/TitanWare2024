package frc.robot.constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.utils.PoseUtils;

import java.util.Optional;

public class FieldConstants {
    public static final double FIELD_LENGTH_X_METERS = Units.inchesToMeters(651.223);
    public static final double FIELD_WIDTH_Y_METERS = Units.inchesToMeters(323.277);

    public static final Pose2d BLUE_SPEAKER_POSE =
            new Pose2d(new Translation2d(0.2381, 5.547868), Rotation2d.fromDegrees(0));

    public static final Pose2d RED_SPEAKER_POSE =
            new Pose2d(new Translation2d(16.379342, 5.547868), Rotation2d.fromDegrees(180));

    public static final Pose2d BLUE_AMP_POSE =
            new Pose2d(
                    Units.inchesToMeters(72.455),
                    FIELD_WIDTH_Y_METERS - Units.inchesToMeters(20),
                    Rotation2d.fromRadians(-Math.PI / 2)
            );

    public static final Pose2d RED_AMP_POSE = PoseUtils.flip(BLUE_AMP_POSE);

    private static Pose2d getAllianceFlippedPose(final Pose2d blueAlliancePose, final Pose2d redAlliancePose) {
        final Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            if (alliance.get() == DriverStation.Alliance.Red) {
                return redAlliancePose;
            } else {
                return blueAlliancePose;
            }
        } else {
            // return blue side pose if unknown alliance
            return blueAlliancePose;
        }
    }

    public static Pose2d getAmpScoringPose() {
        return getAllianceFlippedPose(BLUE_AMP_POSE, RED_AMP_POSE);
    }

    public static Pose2d getSpeakerPose() {
        return getAllianceFlippedPose(BLUE_SPEAKER_POSE, RED_SPEAKER_POSE);
    }
}
