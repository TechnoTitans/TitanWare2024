package frc.robot.utils.teleop;

import edu.wpi.first.math.util.Units;
import frc.robot.constants.Constants;

public class Profiler {
    private static DriverProfile driverProfile = DriverProfile.DEFAULT;
    private static SwerveSpeed swerveSpeed = SwerveSpeed.NORMAL;

    private Profiler() {}

    public static DriverProfile getDriverProfile() {
        return driverProfile;
    }

    public static void setDriverProfile(final DriverProfile driverProfile) {
        if (driverProfile != null) {
            Profiler.driverProfile = driverProfile;
        }
    }

    public static SwerveSpeed getSwerveSpeed() {
        return swerveSpeed;
    }

    public static void setSwerveSpeed(final SwerveSpeed swerveSpeed) {
        Profiler.swerveSpeed = swerveSpeed;
    }

    public enum DriverProfile {
        DRIVER1(1, 1),
        DRIVER2(1.1, 1.1),
        DEFAULT(1, 1);

        final double throttleSensitivity;
        final double rotationalSensitivity;

        DriverProfile(final double throttleSensitivity, final double rotationalSensitivity) {
            this.throttleSensitivity = throttleSensitivity;
            this.rotationalSensitivity = rotationalSensitivity;
        }

        public double getThrottleSensitivity() {
            return throttleSensitivity;
        }

        public double getRotationalSensitivity() {
            return rotationalSensitivity;
        }
    }

    public enum SwerveSpeed {
        FAST(Units.feetToMeters(16.5), 0.75 * Math.PI),
        NORMAL(Units.feetToMeters(10), 0.25 * Math.PI),
        SLOW(Units.feetToMeters(2), 0.1 * Math.PI);

        final double throttleWeight;
        final double rotateWeight;

        SwerveSpeed(final double throttleWeight, final double rotateWeight) {
            this.throttleWeight = throttleWeight / Constants.Swerve.TELEOP_MAX_SPEED_MPS;
            this.rotateWeight = (Math.PI * rotateWeight) / Constants.Swerve.TELEOP_MAX_ANGULAR_SPEED_RAD_PER_SEC;
        }

        public double getThrottleWeight() {
            return throttleWeight;
        }

        public double getRotateWeight() {
            return rotateWeight;
        }
    }
}
