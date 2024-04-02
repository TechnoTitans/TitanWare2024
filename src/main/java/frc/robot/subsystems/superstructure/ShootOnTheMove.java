package frc.robot.subsystems.superstructure;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

import java.util.function.Function;

public class ShootOnTheMove {
    private static final int MAX_ITERATIONS = 3;
    public record Shot(ShotParameters.Parameters parameters, Pose2d futurePose) {}

    public static Shot calculate(
            final Pose2d currentPose,
            final ChassisSpeeds chassisSpeeds,
            final Function<Pose2d, ShotParameters.Parameters> parametersFunction
    ) {
        Pose2d futurePose = currentPose;

        ShotParameters.Parameters parameters = parametersFunction.apply(currentPose);
        double timeOfFlightSeconds = parameters.shotTimeSeconds();

        for (int i = 0; i < MAX_ITERATIONS; i++) {
            final Twist2d twist = new Twist2d(
                    chassisSpeeds.vxMetersPerSecond * timeOfFlightSeconds,
                    chassisSpeeds.vyMetersPerSecond * timeOfFlightSeconds,
                    chassisSpeeds.omegaRadiansPerSecond * timeOfFlightSeconds
            );

            futurePose = currentPose.exp(twist);
            parameters = parametersFunction.apply(futurePose);
            timeOfFlightSeconds = parameters.shotTimeSeconds();
        }

        return new Shot(parameters, futurePose);
    }
}
