package frc.robot.subsystems.superstructure;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

import java.util.function.Function;
import java.util.function.Supplier;

public class ShootOnTheMove {
    private static final int MAX_ITERATIONS = 3;
    public record Shot(ShotParameters.Parameters parameters, Pose2d futurePose) {}

    public Shot calculate(
            final Supplier<Pose2d> poseSupplier,
            final Supplier<ChassisSpeeds> chassisSpeedsSupplier,
            final Function<Pose2d, ShotParameters.Parameters> parametersFunction
    ) {
        final Pose2d initialPose = poseSupplier.get();
        Pose2d futurePose = initialPose;

        ShotParameters.Parameters parameters = parametersFunction.apply(initialPose);
        double timeOfFlightSeconds = parameters.shotTimeSeconds();

        for (int i = 0; i < MAX_ITERATIONS; i++) {
            final ChassisSpeeds chassisSpeeds = chassisSpeedsSupplier.get();
            final Twist2d twist = new Twist2d(
                    chassisSpeeds.vxMetersPerSecond * timeOfFlightSeconds,
                    chassisSpeeds.vyMetersPerSecond * timeOfFlightSeconds,
                    chassisSpeeds.omegaRadiansPerSecond * timeOfFlightSeconds
            );

            final Pose2d currentPose = poseSupplier.get();
            futurePose = currentPose.exp(twist);

            parameters = parametersFunction.apply(futurePose);
            timeOfFlightSeconds = parameters.shotTimeSeconds();
        }

        return new Shot(parameters, futurePose);
    }
}