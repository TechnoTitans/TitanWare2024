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
        double WHEEL_BASE_M = Units.inchesToMeters(17.161);
        double TRACK_WIDTH_M = Units.inchesToMeters(24.752);
        double ROBOT_MAX_SPEED_MPS = Units.feetToMeters(16.5);
        double ROBOT_MAX_ANGULAR_SPEED_RAD_PER_SEC = 2 * Math.PI;
        double TELEOP_MAX_SPEED_MPS = ROBOT_MAX_SPEED_MPS;
        double TELEOP_MAX_ANGULAR_SPEED_RAD_PER_SEC = ROBOT_MAX_ANGULAR_SPEED_RAD_PER_SEC;

        interface Modules {
            double WHEEL_RADIUS_M = 0.0508; //2 in
            double WHEEL_MASS_KG = 0.2313321; //0.51 lbs
            double DRIVE_WHEEL_MOMENT_OF_INERTIA_KG_M_SQUARED = WHEEL_MASS_KG * WHEEL_RADIUS_M * WHEEL_RADIUS_M;
            double TURN_WHEEL_MOMENT_OF_INERTIA_KG_M_SQUARED = 0.004;
            double DRIVER_GEAR_RATIO = 6.122;
            double TURNER_GEAR_RATIO = 150.0 / 7.0;

            //TODO: TUNE AT DE
            double SLIP_CURRENT_A = 80;
            double COUPLING_GEAR_RATIO = 50d / 14;

            double WHEEL_CIRCUMFERENCE_M = 2 * Math.PI * WHEEL_RADIUS_M;
            // SDS MK4i L3 with Kraken X60 FOC
            // see https://www.swervedrivespecialties.com/products/mk4i-swerve-module
            double MODULE_MAX_SPEED_M_PER_SEC = Units.feetToMeters(16.5);

            /** Simulated drive voltage required to overcome friction. */
            double DRIVE_KS_VOLTS = 0.25;
            /** Simulated steer voltage required to overcome friction. */
            double STEER_KS_VOLTS = 0.25;
        }
    }

    interface Intake {
        double RollerRadiusMeters = Units.inchesToMeters(1);
        double RollerCircumferenceMeters = Math.PI * RollerRadiusMeters * 2;
    }

    interface NetworkTables {
        boolean USE_STRUCT_AND_PROTOBUF = true;
    }

    interface Vision {
        PhotonPoseEstimator.PoseStrategy MULTI_TAG_POSE_STRATEGY =
                PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR;

        Transform3d ROBOT_TO_FR_APRILTAG_CAM = new Transform3d(
                new Translation3d(Units.inchesToMeters(11.838), Units.inchesToMeters(-12.861), Units.inchesToMeters(8.947)),
                new Rotation3d(Units.degreesToRadians(7.44802852475369), Units.degreesToRadians(-25), Units.degreesToRadians(-25))
        );

        //L = Left, R = Right, F = Forward, B = Backward (Facing)
        Transform3d ROBOT_TO_FL_APRILTAG_CAM = new Transform3d( // X Z Y
                new Translation3d(Units.inchesToMeters(11.862), Units.inchesToMeters(12.681), Units.inchesToMeters(8.947)),
                new Rotation3d(Units.degreesToRadians(172.57530061265695), Units.degreesToRadians(-25), Units.degreesToRadians(25))
        );

        /**
         * Standard deviations of the supplied pose estimate (before vision, likely to be solely wheel odometry)
         */
        Vector<N3> STATE_STD_DEVS = VecBuilder.fill(0.1, 0.1, Units.degreesToRadians(5));
        Vector<N3> VISION_MEASUREMENT_STD_DEVS = VecBuilder.fill(0.9, 0.9, Units.degreesToRadians(80));
        double MULTI_TAG_MAX_AMBIGUITY = 0.3;
        double SINGLE_TAG_MAX_AMBIGUITY = 0.2;
    }
}
