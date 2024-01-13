package frc.robot.subsystems.drive;

import com.ctre.phoenix6.signals.NeutralModeValue;
import org.littletonrobotics.junction.AutoLog;

public interface SwerveModuleIO {
    @AutoLog
    class SwerveModuleIOInputs {
        /**
         * Drive wheel (mechanism) position, in rotations
         */
        public double drivePositionRots = 0.0;
        /**
         * Drive wheel (mechanism) velocity, in rots/sec
         */
        public double driveVelocityRotsPerSec = 0.0;
        /**
         * Torque output current of the drive motor (rotor), in Amps
         */
        public double driveTorqueCurrentAmps = 0.0;
        /**
         * Stator current of the drive motor (rotor), in Amps
         */
        public double driveStatorCurrentAmps = 0.0;
        /**
         * Temperature of the drive motor, in Celsius
         */
        public double driveTempCelsius = 0.0;

        /**
         * Turn wheel (mechanism) absolute position, in rotations
         */
        public double turnAbsolutePositionRots = 0.0;
        /**
         * Turn wheel (mechanism) velocity, in rots/sec
         */
        public double turnVelocityRotsPerSec = 0.0;
        /**
         * Torque output current of the turn motor (rotor), in Amps
         */
        public double turnTorqueCurrentAmps = 0.0;
        /**
         * Stator current of the turn motor (rotor), in Amps
         */
        public double turnStatorCurrentAmps = 0.0;
        /**
         * Temperature of the turn motor, in Celsius
         */
        public double turnTempCelsius = 0.0;

        public double[] odometryDrivePositionsRots = new double[0];
        public double[] odometryTurnPositionRots = new double[0];
    }

    /**
     * Updates the set of loggable inputs.
     * @param inputs Logged class of IOInputs
     * @see SwerveModuleIOInputs
     * @see AutoLog
     */
    default void updateInputs(final SwerveModuleIOInputs inputs) {}

    /**
     * Periodic call, does <b>NOT</b> call updateInputs
     */
    default void periodic() {}

    /**
     * Config motors call, should only be invoked once on initialize
     */
    default void config() {}

    /**
     * Set the desired inputs of the {@link SwerveModuleIO}
     * @param desiredDriverVelocity the desired driver motor velocity (in mechanism rots/sec)
     * @param desiredTurnerRotations the desired turner motor rotations (in mechanism rotations)
     */
    default void setInputs(final double desiredDriverVelocity, final double desiredTurnerRotations) {}

    /**
     * Characterize the driving motor by applying a voltage, while holding/keeping the turning motor at a constant angle
     * @param driveVolts volts to apply to the drive motor
     * @param desiredTurnerRotations rotations to hold the turn motor at
     */
    default void setDriveCharacterizationVolts(final double driveVolts, final double desiredTurnerRotations) {}

    /**
     * Characterize the driving motor by applying a torque current, while holding/keeping the turning motor at a
     * constant angle
     * @param driveTorqueCurrentAmps torque current amps to apply to the drive motor
     * @param desiredTurnerRotations rotations to hold the turn motor at
     */
    default void setDriveCharacterizationAmps(final double driveTorqueCurrentAmps, final double desiredTurnerRotations) {}

    /**
     * Set the desired {@link NeutralModeValue} of the drive motor on this module
     * @param neutralMode the desired {@link NeutralModeValue}
     * @see NeutralModeValue
     */
    default void setNeutralMode(final NeutralModeValue neutralMode) {}
}
