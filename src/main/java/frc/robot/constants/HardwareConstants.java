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
            -0.137
    );

    public static final SwerveModuleConstants FRONT_RIGHT_MODULE = new SwerveModuleConstants(
            "FrontRight",
            RobotMap.CanivoreCANBus,
            new Translation2d(WHEEL_BASE_M / 2, -TRACK_WIDTH_M / 2),
            4,
            5,
            6,
            0.381
    );

    public static final SwerveModuleConstants BACK_LEFT_MODULE = new SwerveModuleConstants(
            "BackLeft",
            RobotMap.CanivoreCANBus,
            new Translation2d(-WHEEL_BASE_M / 2, TRACK_WIDTH_M / 2),
            7,
            8,
            9,
            0
    );

    public static final SwerveModuleConstants BACK_RIGHT_MODULE = new SwerveModuleConstants(
            "BackRight",
            RobotMap.CanivoreCANBus,
            new Translation2d(-WHEEL_BASE_M / 2, -TRACK_WIDTH_M / 2),
            10,
            11,
            12,
            -0.429
    );

    public record GyroConstants(
            String CANBus,
            int gyroId
    ) {}

    public static final GyroConstants GYRO = new GyroConstants(
            RobotMap.CanivoreCANBus,
            13
    );
  
    public record ArmConstants(
            String CANBus,
            int leftPivotMotorId,
            int rightPivotMotorId,
            int pivotCANCoderId,
            double pivotGearing,
            double pivotCANCoderOffset,
            double pivotSoftLowerLimitRots,
            double pivotSoftUpperLimitRots
    ) {}

    public static final ArmConstants ARM = new ArmConstants(
            RobotMap.CanivoreCANBus,
            14,
            15,
            16,
            112.5,
            0.221,
            0.05,
            0.27
    );

    public record ShooterConstants(
            String CANBus,
            int ampMotorId,
            double ampMotorGearing,
            int leftFlywheelMotorId,
            double leftFlywheelGearing,
            int rightFlywheelMotorId,
            double rightFlywheelGearing
    ) {}

    public static final ShooterConstants SHOOTER = new ShooterConstants(
            RobotMap.RioCANBus,
            17,
            0.5,
            18,
            0.5,
            19,
            0.5
    );

    public record IntakeConstants(
            String CANBus,
            int rightRollerMotor,
            int leftRollerMotor,
            int shooterFeederRollerMotor,
            int sensorDigitalInput,
            double rightMotorGearing,
            double leftMotorGearing,
            double shooterFeederMotorGearing
    ) {}

    public static final IntakeConstants INTAKE = new IntakeConstants(
            RobotMap.RioCANBus,
            20,
            21,
            22,
            1,
            2,
            2,
            2
    );
}
