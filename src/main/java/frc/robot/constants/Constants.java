package frc.robot.constants;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import org.photonvision.PhotonPoseEstimator;

public interface Constants {
    RobotMode CURRENT_MODE = RobotMode.REAL;
    CompetitionType CURRENT_COMPETITION_TYPE = CompetitionType.TESTING;
    double LOOP_PERIOD_SECONDS = 0.02;

    enum RobotMode {
        REAL,
        SIM,
        REPLAY
    }

    enum CompetitionType {
        TESTING,
        COMPETITION
    }

    interface NetworkTables {
        String AUTO_TABLE = "AutoSelector";
        String AUTO_PUBLISHER = "AutoOptions";
        String AUTO_SELECTED_SUBSCRIBER = "SelectedAuto";
    }

    interface Vision {
        PhotonPoseEstimator.PoseStrategy MULTI_TAG_POSE_STRATEGY =
                PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR;

        Transform3d ROBOT_TO_FL_APRILTAG = new Transform3d(
                new Translation3d(Units.inchesToMeters(11.862), Units.inchesToMeters(12.681), Units.inchesToMeters(8.947)),
                new Rotation3d(0, Units.degreesToRadians(-25), Units.degreesToRadians(20))
        );

        Transform3d ROBOT_TO_FC_APRILTAG = new Transform3d(
                new Translation3d(Units.inchesToMeters(13.36613), Units.inchesToMeters(8.5202), Units.inchesToMeters(6.759)),
                new Rotation3d(0, Units.degreesToRadians(-30), 0)
        );

        Transform3d ROBOT_TO_FR_APRILTAG = new Transform3d(
                new Translation3d(Units.inchesToMeters(11.838), Units.inchesToMeters(-12.861), Units.inchesToMeters(8.947)),
                new Rotation3d(0, Units.degreesToRadians(-25), Units.degreesToRadians(-20))
        );

        Transform3d ROBOT_TO_REAR_NOTED = new Transform3d(
                new Translation3d(Units.inchesToMeters(-14), Units.inchesToMeters(0), Units.inchesToMeters(16)),
                new Rotation3d(0, Units.degreesToRadians(25), Units.degreesToRadians(180))
        );

        /**
         * Standard deviations of the supplied pose estimate (before vision, likely to be solely wheel odometry)
         */
        Vector<N3> STATE_STD_DEVS = VecBuilder.fill(0.1, 0.1, Units.degreesToRadians(1));
        double VISION_CAMERA_DEFAULT_STD_DEV_FACTOR = 1.0;
        Vector<N3> VISION_STD_DEV_COEFFS = VecBuilder.fill(0.02, 0.02, 0.02);
        double MULTI_TAG_MAX_AMBIGUITY = 0.4;
        double SINGLE_TAG_MAX_AMBIGUITY = 0.2;
    }
}
