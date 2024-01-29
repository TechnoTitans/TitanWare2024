package frc.robot.constants;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.drive.SwerveModule;

import static frc.robot.constants.Constants.Swerve.TRACK_WIDTH_M;
import static frc.robot.constants.Constants.Swerve.WHEEL_BASE_M;

public class HardwareConstants {
    public record SwerveModuleConstants(
            String name,
            String moduleCANBus,
            Hardware hardware,
            Translation2d translationOffset,
            int driveMotorId,
            int turnMotorId,
            int turnEncoderId,
            double turnEncoderOffset
    ) {
        public enum Hardware {
            SDSMk4i_TalonFX_CANCoder
        }

        public static SwerveModule create(
                final SwerveModuleConstants swerveModuleConstants,
                final Constants.RobotMode currentMode
        ) {
            return switch(swerveModuleConstants.hardware) {
                case SDSMk4i_TalonFX_CANCoder -> SwerveModule.Builder.SDSMK4iTalonFXCANCoder(
                        swerveModuleConstants.name,
                        new TalonFX(swerveModuleConstants.driveMotorId, swerveModuleConstants.moduleCANBus),
                        new TalonFX(swerveModuleConstants.turnMotorId, swerveModuleConstants.moduleCANBus),
                        new CANcoder(swerveModuleConstants.turnEncoderId, swerveModuleConstants.moduleCANBus),
                        swerveModuleConstants.turnEncoderOffset,
                        currentMode
                );
            };
        }

        public SwerveModule create(final Constants.RobotMode currentMode) {
            return SwerveModuleConstants.create(this, currentMode);
        }
    }

    public static final SwerveModuleConstants FRONT_LEFT_MODULE = new SwerveModuleConstants(
            "FrontLeft",
            RobotMap.CanivoreCANBus,
            SwerveModuleConstants.Hardware.SDSMk4i_TalonFX_CANCoder,
            new Translation2d(WHEEL_BASE_M / 2, TRACK_WIDTH_M / 2),
            1,
            2,
            3,
            0
    );

    public static final SwerveModuleConstants FRONT_RIGHT_MODULE = new SwerveModuleConstants(
            "FrontRight",
            RobotMap.CanivoreCANBus,
            SwerveModuleConstants.Hardware.SDSMk4i_TalonFX_CANCoder,
            new Translation2d(WHEEL_BASE_M / 2, -TRACK_WIDTH_M / 2),
            4,
            5,
            6,
            0
    );

    public static final SwerveModuleConstants BACK_LEFT_MODULE = new SwerveModuleConstants(
            "BackLeft",
            RobotMap.CanivoreCANBus,
            SwerveModuleConstants.Hardware.SDSMk4i_TalonFX_CANCoder,
            new Translation2d(-WHEEL_BASE_M / 2, TRACK_WIDTH_M / 2),
            7,
            8,
            9,
            0
    );

    public static final SwerveModuleConstants BACK_RIGHT_MODULE = new SwerveModuleConstants(
            "BackRight",
            RobotMap.CanivoreCANBus,
            SwerveModuleConstants.Hardware.SDSMk4i_TalonFX_CANCoder,
            new Translation2d(-WHEEL_BASE_M / 2, -TRACK_WIDTH_M / 2),
            10,
            11,
            12,
            0
    );

    public record ShooterConstants(
            String CANbus,
            int topMotorId,
            int bottomMotorId
    ) {}

    public static final ShooterConstants SHOOTER = new ShooterConstants(
            RobotMap.RioCANBus,
            13,
            14
    );
}
