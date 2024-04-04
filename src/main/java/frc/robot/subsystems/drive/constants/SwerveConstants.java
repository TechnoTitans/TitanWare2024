package frc.robot.subsystems.drive.constants;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.constants.Constants;
import frc.robot.constants.RobotMap;
import frc.robot.subsystems.drive.OdometryThreadRunner;
import frc.robot.subsystems.drive.SwerveModule;

public class SwerveConstants {
    public static final SwerveConfig Config = new SwerveConfig(
            0.0508,
            6.122,
            150.0 / 7.0,
            50.0 / 14.0,
            Units.inchesToMeters(17.161),
            Units.inchesToMeters(24.752),
            Units.feetToMeters(15.5),
            Units.feetToMeters(80.0),
            12,
            6
    );

    public static final SwerveModuleConstants FrontLeftModule = new SwerveModuleConstants(
            "FrontLeft",
            RobotMap.CanivoreCANBus,
            new Translation2d(Config.wheelBaseMeters / 2, Config.trackWidthMeters / 2),
            1,
            2,
            3,
            -0.137
    );

    public static final SwerveModuleConstants FrontRightModule = new SwerveModuleConstants(
            "FrontRight",
            RobotMap.CanivoreCANBus,
            new Translation2d(Config.wheelBaseMeters / 2, -Config.trackWidthMeters / 2),
            4,
            5,
            6,
            0.381
    );

    public static final SwerveModuleConstants BackLeftModule = new SwerveModuleConstants(
            "BackLeft",
            RobotMap.CanivoreCANBus,
            new Translation2d(-Config.wheelBaseMeters / 2, Config.trackWidthMeters / 2),
            7,
            8,
            9,
            0
    );

    public static final SwerveModuleConstants BackRightModule = new SwerveModuleConstants(
            "BackRight",
            RobotMap.CanivoreCANBus,
            new Translation2d(-Config.wheelBaseMeters / 2, -Config.trackWidthMeters / 2),
            10,
            11,
            12,
            -0.429
    );

    public record SwerveConfig(
            double wheelRadiusMeters,
            double driveReduction,
            double turnReduction,
            double couplingRatio,
            double wheelBaseMeters,
            double trackWidthMeters,
            double maxLinearVelocity,
            double maxLinearAcceleration,
            double maxAngularVelocity,
            double maxAngularAcceleration
    ) {
        public double driveBaseRadiusMeters() {
            return Math.hypot(wheelBaseMeters / 2, trackWidthMeters / 2);
        }

        public double wheelCircumferenceMeters() {
            return 2 * Math.PI * wheelRadiusMeters;
        }
    }

    public record SwerveModuleConstants(
            String name,
            String moduleCANBus,
            Translation2d translationOffset,
            int driveMotorId,
            int turnMotorId,
            int turnEncoderId,
            double turnEncoderOffsetRots
    ) {
        public static SwerveModule create(
                final SwerveModuleConstants constants,
                final OdometryThreadRunner odometryThreadRunner,
                final Constants.RobotMode robotMode
        ) {
            return new SwerveModule(constants, odometryThreadRunner, robotMode);
        }

        public SwerveModule create(
                final Constants.RobotMode robotMode,
                final OdometryThreadRunner odometryThreadRunner
        ) {
            return SwerveModuleConstants.create(this, odometryThreadRunner, robotMode);
        }
    }
}
