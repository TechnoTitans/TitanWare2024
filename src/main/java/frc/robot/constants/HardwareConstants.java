package frc.robot.constants;

public class HardwareConstants {
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
            0.900634765625,
            0.01,
            0.264
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
