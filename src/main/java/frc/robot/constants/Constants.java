package frc.robot.constants;

import edu.wpi.first.math.util.Units;

public interface Constants {
    RobotMode CURRENT_MODE = RobotMode.REAL;
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
            double SLIP_CURRENT_A = 55;
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
}
