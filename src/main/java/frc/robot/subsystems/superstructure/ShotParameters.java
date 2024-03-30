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
            double ampVelocityRotsPerSec
    ) implements Interpolatable<Parameters> {
        public static final Interpolator<Parameters> interpolator = Parameters::interpolate;

        @Override
        public Parameters interpolate(final Parameters endValue, final double t) {
            return new Parameters(
                    this.armPivotAngle.interpolate(endValue.armPivotAngle, t),
                    MathUtil.interpolate(this.leftVelocityRotsPerSec, endValue.leftVelocityRotsPerSec, t),
                    MathUtil.interpolate(this.rightVelocityRotsPerSec, endValue.rightVelocityRotsPerSec, t),
                    MathUtil.interpolate(this.ampVelocityRotsPerSec, endValue.ampVelocityRotsPerSec, t)
            );
        }
    }

    private static final InterpolatingTreeMap<Double, Parameters> shotDataMap = new InterpolatingTreeMap<>(
            InverseInterpolator.forDouble(),
            Parameters.interpolator
    );

    static {
        shotDataMap.put(1.5d, new Parameters(
                Rotation2d.fromDegrees(52.5),
                133.3,
                100,
                100
        ));

        shotDataMap.put(2.15d, new Parameters(
                Rotation2d.fromDegrees(44),
                133.3,
                100,
                100
        ));

        shotDataMap.put(2.5d, new Parameters(
                Rotation2d.fromDegrees(38.5),
                133.3,
                100,
                100
        ));

        shotDataMap.put(3d, new Parameters(
                Rotation2d.fromDegrees(35),
                133.3,
                100,
                100
        ));

        shotDataMap.put(3.5d, new Parameters(
                Rotation2d.fromDegrees(32.75),
                133.3,
                100,
                100
        ));

        shotDataMap.put(3.6d, new Parameters(
                Rotation2d.fromDegrees(33.5),
                133.3,
                100,
                100
        ));

        shotDataMap.put(4d, new Parameters(
                Rotation2d.fromDegrees(29.5),
                133.3,
                100,
                100
        ));

        shotDataMap.put(4.5d, new Parameters(
                Rotation2d.fromDegrees(28),
                133.3,
                100,
                100
        ));

        shotDataMap.put(5d, new Parameters(
                Rotation2d.fromDegrees(27.75),
                133.3,
                100,
                100
        ));

        shotDataMap.put(5.5d, new Parameters(
                Rotation2d.fromDegrees(25.5),
                133.3,
                100,
                100
        ));

        shotDataMap.put(6d, new Parameters(
                Rotation2d.fromDegrees(24.75),
                133.3,
                100,
                100
        ));
    }

    public static Parameters get(final double distanceMeters) {
        return shotDataMap.get(distanceMeters);
    }
}
