package frc.robot.utils.gyro;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.*;
import frc.robot.constants.Constants;

import java.util.function.Supplier;

public class GyroUtils {
    public static Rotation2d withAngleModulus(final Supplier<Rotation2d> rotation2dSupplier) {
        return Rotation2d.fromRadians(MathUtil.angleModulus(rotation2dSupplier.get().getRadians()));
    }

    public static double getAsDoubleDeg(final Rotation2d rotation2d) {
        return rotation2d.getDegrees();
    }

    public static double getAsDoubleDeg(final Supplier<Rotation2d> rotation2dSupplier) {
        return getAsDoubleDeg(rotation2dSupplier.get());
    }

    public static double getAsAngleModdedDoubleDeg(final Supplier<Rotation2d> rotation2dSupplier) {
        return getAsDoubleDeg(withAngleModulus(rotation2dSupplier));
    }

    public static Pose3d robotPose2dToPose3dWithGyro(final Pose2d pose2d, final Rotation3d gyroRotation) {
        return new Pose3d(pose2d)
                .exp(new Twist3d(
                        0, 0, Math.abs(gyroRotation.getY()) * Constants.Swerve.WHEEL_BASE_M * 0.5,
                        0, gyroRotation.getY(), 0
                ))
                .exp(new Twist3d(
                        0, 0, Math.abs(gyroRotation.getX()) * Constants.Swerve.TRACK_WIDTH_M * 0.5,
                        gyroRotation.getX(), 0, 0
                ));
    }

    /**
     * Convert a Roll, Pitch, and Yaw into a {@link Rotation3d} object.
     * @param roll the roll (rad)
     * @param pitch the pitch (rad)
     * @param yaw the yaw (rad)
     * @return the {@link Rotation3d} describing the supplied euler angles
     * @see Rotation3d
     */
    public static Rotation3d rpyToRotation3d(final double roll, final double pitch, final double yaw) {
        return new Rotation3d(roll, pitch, yaw);
    }

    /**
     * Convert Roll, Pitch, and Yaw {@link Rotation2d}s into a {@link Rotation3d} object.
     * @param roll the roll {@link Rotation2d}
     * @param pitch the pitch {@link Rotation2d}
     * @param yaw the yaw {@link Rotation2d}
     * @return the {@link Rotation3d} describing the supplied euler angles
     * @see Rotation3d
     */
    public static Rotation3d rpyToRotation3d(final Rotation2d roll, final Rotation2d pitch, final Rotation2d yaw) {
        return rpyToRotation3d(roll.getRadians(), pitch.getRadians(), yaw.getRadians());
    }
}
