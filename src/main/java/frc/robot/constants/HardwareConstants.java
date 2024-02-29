package frc.robot.constants;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.drive.OdometryThreadRunner;
import frc.robot.subsystems.drive.SwerveModule;

import static frc.robot.constants.Constants.Swerve.TRACK_WIDTH_M;
import static frc.robot.constants.Constants.Swerve.WHEEL_BASE_M;

public class HardwareConstants {
    public record SwerveModuleConstants(
            String name,
            String moduleCANBus,
            Translation2d translationOffset,
            int driveMotorId,
            int turnMotorId,
            int turnEncoderId,
            double turnEncoderOffset
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

    public static final SwerveModuleConstants FRONT_LEFT_MODULE = new SwerveModuleConstants(
            "FrontLeft",
            RobotMap.CanivoreCANBus,
            new Translation2d(WHEEL_BASE_M / 2, TRACK_WIDTH_M / 2),
            1,
            2,
            3,
            0.320556640625
    );

    public static final SwerveModuleConstants FRONT_RIGHT_MODULE = new SwerveModuleConstants(
            "FrontRight",
            RobotMap.CanivoreCANBus,
            new Translation2d(WHEEL_BASE_M / 2, -TRACK_WIDTH_M / 2),
            4,
            5,
            6,
            0.33251953125
    );

    public static final SwerveModuleConstants BACK_LEFT_MODULE = new SwerveModuleConstants(
            "BackLeft",
            RobotMap.CanivoreCANBus,
            new Translation2d(-WHEEL_BASE_M / 2, TRACK_WIDTH_M / 2),
            7,
            8,
            9,
            0.0478515625
    );

    public static final SwerveModuleConstants BACK_RIGHT_MODULE = new SwerveModuleConstants(
            "BackRight",
            RobotMap.CanivoreCANBus,
            new Translation2d(-WHEEL_BASE_M / 2, -TRACK_WIDTH_M / 2),
            10,
            11,
            12,
            0.283203125
    );

    public record GyroConstants(
            String CANBus,
            int gyroId
    ) {}

    public static final GyroConstants GYRO = new GyroConstants(
            RobotMap.CanivoreCANBus,
            13
    );
}
