package frc.robot.subsystems.superstructure;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.Interpolatable;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import frc.robot.constants.FieldConstants;

import java.util.function.Supplier;

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
                Rotation2d.fromDegrees(35),
                84.433/2,
                128.883/2,
                84.433/2,
                0
        ));
    }

    private static final InterpolatingTreeMap<Double, Parameters> ferryDataMap = new InterpolatingTreeMap<>(
            InverseInterpolator.forDouble(),
            Parameters.interpolator
    );

    static {
        ferryDataMap.put(6d, new Parameters(
                Rotation2d.fromDegrees(50),
                40,
                60,
                40,
                0
        ));

        ferryDataMap.put(6.5d, new Parameters(
                Rotation2d.fromDegrees(50),
                40,
                60,
                40,
                0
        ));

        ferryDataMap.put(7d, new Parameters(
                Rotation2d.fromDegrees(50),
                40,
                60,
                40,
                0
        ));

        ferryDataMap.put(7.5d, new Parameters(
                Rotation2d.fromDegrees(50),
                50,
                70,
                50,
                0
        ));

        ferryDataMap.put(8d, new Parameters(
                Rotation2d.fromDegrees(50),
                55,
                75,
                55,
                0
        ));

        ferryDataMap.put(8.5d, new Parameters(
                Rotation2d.fromDegrees(50),
                55,
                75,
                55,
                0
        ));

        ferryDataMap.put(9d, new Parameters(
                Rotation2d.fromDegrees(50),
                55,
                75,
                55,
                0
        ));

        ferryDataMap.put(9.5d, new Parameters(
                Rotation2d.fromDegrees(50),
                60,
                80,
                60,
                0
        ));

        ferryDataMap.put(10d, new Parameters(
                Rotation2d.fromDegrees(50),
                62.5,
                82.5,
                62.5,
                0
        ));

        ferryDataMap.put(10.5d, new Parameters(
                Rotation2d.fromDegrees(50),
                65,
                85,
                65,
                0
        ));

        ferryDataMap.put(11d, new Parameters(
                Rotation2d.fromDegrees(50),
                65,
                85,
                65,
                0
        ));
    }

    public static Parameters getShotParameters(final Pose2d currentPose) {
        return shotDataMap.get(
                currentPose
                        .minus(FieldConstants.getSpeakerPose())
                        .getTranslation()
                        .getNorm()
        );
    }

    public static Supplier<Parameters> shotParametersSupplier(
            final Supplier<Pose2d> currentPoseSupplier
    ) {
        return () -> ShotParameters.getShotParameters(currentPoseSupplier.get());
    }

    public static Parameters getFerryParameters(final Pose2d currentPose) {
        return ferryDataMap.get(
                currentPose
                        .minus(FieldConstants.getFerryPose())
                        .getTranslation()
                        .getNorm()
        );
    }
}
