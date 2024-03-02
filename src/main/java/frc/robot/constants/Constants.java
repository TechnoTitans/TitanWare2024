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
    RobotMode CURRENT_MODE = RobotMode.SIM;
    double LOOP_PERIOD_SECONDS = 0.02;

    enum RobotMode {
        REAL,
        SIM,
        REPLAY
    }

    interface Swerve {
        double WHEEL_BASE_M = 0.7366;
        double TRACK_WIDTH_M = 0.7366;
//        double ROBOT_MAX_SPEED_MPS = Units.feetToMeters(16.5);
        double ROBOT_MAX_SPEED_MPS = Units.feetToMeters(13.5);
        double ROBOT_MAX_ANGULAR_SPEED_RAD_PER_SEC = 2 * Math.PI;
        double TELEOP_MAX_SPEED_MPS = ROBOT_MAX_SPEED_MPS;
        double TELEOP_MAX_ANGULAR_SPEED_RAD_PER_SEC = ROBOT_MAX_ANGULAR_SPEED_RAD_PER_SEC;

        interface Modules {
            double WHEEL_RADIUS_M = 0.0508; //2 in
            double WHEEL_MASS_KG = 0.2313321; //0.51 lbs
            double DRIVE_WHEEL_MOMENT_OF_INERTIA_KG_M_SQUARED = WHEEL_MASS_KG * WHEEL_RADIUS_M * WHEEL_RADIUS_M;
            double TURN_WHEEL_MOMENT_OF_INERTIA_KG_M_SQUARED = 0.004;
//            double DRIVER_GEAR_RATIO = 6.122;
            double DRIVER_GEAR_RATIO = 8.14;
            double TURNER_GEAR_RATIO = 150.0 / 7.0;

            double SLIP_CURRENT_A = 55;
            double COUPLING_GEAR_RATIO = 50d / 14;

            double WHEEL_CIRCUMFERENCE_M = 2 * Math.PI * WHEEL_RADIUS_M;
            // SDS MK4i L3 with Kraken X60 FOC
            // see https://www.swervedrivespecialties.com/products/mk4i-swerve-module
//            double MODULE_MAX_SPEED_M_PER_SEC = Units.feetToMeters(16.5);
            double MODULE_MAX_SPEED_M_PER_SEC = Units.feetToMeters(13.5);

            /** Simulated drive voltage required to overcome friction. */
            double DRIVE_KS_VOLTS = 0.25;
            /** Simulated steer voltage required to overcome friction. */
            double STEER_KS_VOLTS = 0.25;
        }
    }

    interface NetworkTables {
        boolean USE_STRUCT_AND_PROTOBUF = true;
    }

    interface Vision {
        PhotonPoseEstimator.PoseStrategy MULTI_TAG_POSE_STRATEGY =
                PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR;

        //L = Left, R = Right, F = Forward, B = Backward (Facing)
        Transform3d ROBOT_TO_FR_APRILTAG_CAM = new Transform3d(
                new Translation3d(Units.inchesToMeters(13.449), Units.inchesToMeters(-13.762), Units.inchesToMeters(7.922)),
                new Rotation3d(0, Units.degreesToRadians(-15), Units.degreesToRadians(-70))
        );

        Transform3d ROBOT_TO_FL_APRILTAG_CAM = new Transform3d(
                new Translation3d(Units.inchesToMeters(14.465), Units.inchesToMeters(-11.907), Units.inchesToMeters(9.67)),
                new Rotation3d(0, Units.degreesToRadians(-15), Units.degreesToRadians(25))
        );

        /**
         * Standard deviations of the supplied pose estimate (before vision, likely to be solely wheel odometry)
         */
        Vector<N3> STATE_STD_DEVS = VecBuilder.fill(0.2, 0.2, Units.degreesToRadians(2.5));
        Vector<N3> VISION_MEASUREMENT_STD_DEVS = VecBuilder.fill(0.85, 0.85, Units.degreesToRadians(5));
        double MULTI_TAG_MAX_AMBIGUITY = 0.3;
        double SINGLE_TAG_MAX_AMBIGUITY = 0.2;
    }
}
