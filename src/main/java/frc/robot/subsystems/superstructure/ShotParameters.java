package frc.robot.subsystems.superstructure;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.Interpolatable;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.math.interpolation.InverseInterpolator;

public class ShotParameters {
    public record Parameters(
            Rotation2d armPivotAngle,
            double leftVelocityRotsPerSec,
            double rightVelocityRotsPerSec,
            double ampVelocityRotsPerSec,
            double shotTimeSeconds
    ) implements Interpolatable<Parameters> {
        public static final Interpolator<Parameters> interpolator = Parameters::interpolate;

        @Override
        public Parameters interpolate(final Parameters endValue, final double t) {
            return new Parameters(
                    this.armPivotAngle.interpolate(endValue.armPivotAngle, t),
                    MathUtil.interpolate(this.leftVelocityRotsPerSec, endValue.leftVelocityRotsPerSec, t),
                    MathUtil.interpolate(this.rightVelocityRotsPerSec, endValue.rightVelocityRotsPerSec, t),
                    MathUtil.interpolate(this.ampVelocityRotsPerSec, endValue.ampVelocityRotsPerSec, t),
                    MathUtil.interpolate(this.shotTimeSeconds, endValue.shotTimeSeconds, t)
            );
        }
    }

    private static final InterpolatingTreeMap<Double, Parameters> shotDataMap = new InterpolatingTreeMap<>(
            InverseInterpolator.forDouble(),
            Parameters.interpolator
    );

    static {
        shotDataMap.put(1.5d, new Parameters(
                Rotation2d.fromDegrees(49.5),
                84.433,
                128.883,
                84.433,
                0
        ));

        shotDataMap.put(2.15d, new Parameters(
                Rotation2d.fromDegrees(41),
                84.433,
                128.883,
                84.433,
                0.1
        ));

        shotDataMap.put(2.5d, new Parameters(
                Rotation2d.fromDegrees(37),
                84.433,
                128.883,
                84.433,
                0.279
        ));

        shotDataMap.put(3d, new Parameters(
                Rotation2d.fromDegrees(32.5),
                84.433,
                128.883,
                84.433,
                0.325
        ));

        shotDataMap.put(3.5d, new Parameters(
                Rotation2d.fromDegrees(30),
                84.433,
                128.883,
                84.433,
                0.37
        ));

        shotDataMap.put(4d, new Parameters(
                Rotation2d.fromDegrees(26),
                84.433,
                128.883,
                84.433,
                0
        ));

        shotDataMap.put(4.5d, new Parameters(
                Rotation2d.fromDegrees(25.5),
                84.433,
                128.883,
                84.433,
                0
        ));

        shotDataMap.put(5d, new Parameters(
                Rotation2d.fromDegrees(24.75),
                84.433,
                128.883,
                84.433,
                0
        ));

        shotDataMap.put(5.5d, new Parameters(
                Rotation2d.fromDegrees(22),
                100,
                133.3,
                100,
                0
        ));

        shotDataMap.put(6d, new Parameters(
                Rotation2d.fromDegrees(21),
                100,
                133.3,
                100,
                0
        ));

        shotDataMap.put(6.5d, new Parameters(
                Rotation2d.fromDegrees(18.5),
                100,
                133.3,
                100,
                0
        ));

        shotDataMap.put(7d, new Parameters(
                Rotation2d.fromDegrees(17.5),
                100,
                133.3,
                100,
                0
        ));

        shotDataMap.put(7.5d, new Parameters(
                Rotation2d.fromDegrees(16.5),
                100,
                133.3,
                100,
                0
        ));
    }

    public static Parameters get(final double distanceMeters) {
        return shotDataMap.get(distanceMeters);
    }
}
