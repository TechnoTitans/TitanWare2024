package frc.robot.utils.teleop;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;

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

    @SuppressWarnings("unused")
    public enum DriverProfile {
        DRIVER1(1, 1),
        DRIVER2(0.9, 0.9),
        DEFAULT(1, 1);

        final double throttleSensitivity;
        final double rotationalSensitivity;

        DriverProfile(final double throttleSensitivity, final double rotationalSensitivity) {
            this.throttleSensitivity = MathUtil.clamp(throttleSensitivity, 0, 1);
            this.rotationalSensitivity = MathUtil.clamp(rotationalSensitivity, 0, 1);
        }

        public double getTranslationSensitivity() {
            return throttleSensitivity;
        }

        public double getRotationalSensitivity() {
            return rotationalSensitivity;
        }
    }

    public enum SwerveSpeed {
        FAST(Units.feetToMeters(5), 1 * Math.PI),
//        FAST(Units.feetToMeters(15), 2 * Math.PI),
//        NORMAL(Units.feetToMeters(12), 1.5 * Math.PI),
        NORMAL(Units.feetToMeters(4), 0.75 * Math.PI),
        SLOW(Units.feetToMeters(3), 0.2 * Math.PI);

        private final double translationSpeed;
        private final double rotationSpeed;

        SwerveSpeed(final double translationSpeed, final double rotationSpeed) {
            this.translationSpeed = translationSpeed;
            this.rotationSpeed = rotationSpeed;
        }

        public double getTranslationSpeed() {
            return translationSpeed;
        }

        public double getRotationSpeed() {
            return rotationSpeed;
        }
    }
}
