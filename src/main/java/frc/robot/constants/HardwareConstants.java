package frc.robot.constants;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.drive.Swerve;
import frc.robot.subsystems.drive.SwerveModule;

import static frc.robot.constants.Constants.Swerve.*;

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
            SDSMk4i_Falcon500_CANCoder
        }

        public static SwerveModule create(
                final SwerveModuleConstants swerveModuleConstants,
                final Constants.RobotMode currentMode,
                final Swerve.OdometryThreadRunner odometryThreadRunner
                ) {
            return switch(swerveModuleConstants.hardware) {
                case SDSMk4i_Falcon500_CANCoder -> SwerveModule.Builder.SDSMK4iTalonFXCANCoder(
                        swerveModuleConstants.name,
                        new TalonFX(swerveModuleConstants.driveMotorId, swerveModuleConstants.moduleCANBus),
                        new TalonFX(swerveModuleConstants.turnMotorId, swerveModuleConstants.moduleCANBus),
                        new CANcoder(swerveModuleConstants.turnEncoderId, swerveModuleConstants.moduleCANBus),
                        swerveModuleConstants.turnEncoderOffset,
                        currentMode,
                        odometryThreadRunner
                );
            };
        }

        public SwerveModule create(
                final Constants.RobotMode currentMode,
                final Swerve.OdometryThreadRunner odometryThreadRunner
        ) {
            return SwerveModuleConstants.create(this, currentMode, odometryThreadRunner);
        }
    }

    public static final SwerveModuleConstants FRONT_LEFT_MODULE = new SwerveModuleConstants(
            "FrontLeft",
            RobotMap.CanivoreCANBus,
            SwerveModuleConstants.Hardware.SDSMk4i_Falcon500_CANCoder,
            new Translation2d(WHEEL_BASE_M / 2, TRACK_WIDTH_M / 2),
            1,
            2,
            3,
            0.320556640625
    );

    public static final SwerveModuleConstants FRONT_RIGHT_MODULE = new SwerveModuleConstants(
            "FrontRight",
            RobotMap.CanivoreCANBus,
            SwerveModuleConstants.Hardware.SDSMk4i_Falcon500_CANCoder,
            new Translation2d(WHEEL_BASE_M / 2, -TRACK_WIDTH_M / 2),
            4,
            5,
            6,
            0.33251953125
    );

    public static final SwerveModuleConstants BACK_LEFT_MODULE = new SwerveModuleConstants(
            "BackLeft",
            RobotMap.CanivoreCANBus,
            SwerveModuleConstants.Hardware.SDSMk4i_Falcon500_CANCoder,
            new Translation2d(-WHEEL_BASE_M / 2, TRACK_WIDTH_M / 2),
            7,
            8,
            9,
            0.0478515625
    );

    public static final SwerveModuleConstants BACK_RIGHT_MODULE = new SwerveModuleConstants(
            "BackRight",
            RobotMap.CanivoreCANBus,
            SwerveModuleConstants.Hardware.SDSMk4i_Falcon500_CANCoder,
            new Translation2d(-WHEEL_BASE_M / 2, -TRACK_WIDTH_M / 2),
            10,
            11,
            12,
            0.283203125
    );

    public record ElevatorConstants(
            String verticalElevatorCANBus,
            int verticalMainMotorId,
            int verticalFollowerMotorId,
            int verticalEncoderId,
            int verticalLimitSwitchDIOChannel,
            String horizontalElevatorCANBus,
            int horizontalMotorId,
            int horizontalEncoderId,
            int horizontalLimitSwitchDIOChannel
    ) {}

    public static final ElevatorConstants ELEVATOR = new ElevatorConstants(
            RobotMap.CanivoreCANBus,
            14,
            15,
            16,
            6,
            RobotMap.RioCANBus,
            17,
            18,
            8
    );

    public record ClawConstants(
            String clawCANBus,
            int clawMainWheelMotorId,
            int clawFollowerWheelMotorId,
            int clawOpenCloseMotorId,
            int clawOpenCloseEncoderId,
            int clawTiltMotorId,
            int clawTiltEncoderId
    ) {}

    public static final ClawConstants CLAW = new ClawConstants(
            RobotMap.RioCANBus,
            19,
            20,
            21,
            22,
            23,
            24
    );
}
