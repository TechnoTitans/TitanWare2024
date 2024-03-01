package frc.robot.subsystems.drive;

import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.constants.Constants;
import frc.robot.constants.Constants.Swerve.Modules;
import frc.robot.constants.HardwareConstants;
import frc.robot.utils.logging.LogUtils;
import org.littletonrobotics.junction.Logger;

public class SwerveModule {
    private final String name;
    private final String logKey;
    private final SwerveModuleIO moduleIO;
    private final SwerveModuleIOInputsAutoLogged inputs;

    private SwerveModulePosition[] odometryPositions;
    private SwerveModuleState lastDesiredState = new SwerveModuleState();

    public SwerveModule(
            final HardwareConstants.SwerveModuleConstants constants,
            final OdometryThreadRunner odometryThreadRunner,
            final Constants.RobotMode robotMode
    ) {
        this.name = constants.name();
        this.logKey = String.format("%s/%s", Swerve.logKey, name);

        this.moduleIO = switch (robotMode) {
            case REAL -> new SwerveModuleIOTalonFX(constants, odometryThreadRunner);
            case SIM -> new SwerveModuleIOTalonFXSim(constants, odometryThreadRunner);
            case REPLAY -> new SwerveModuleIO() {};
        };
        this.moduleIO.config();

        this.inputs = new SwerveModuleIOInputsAutoLogged();
    }

    public String getName() { return name; }

    public void periodic() {
        final double modulePeriodicUpdateStart = Logger.getRealTimestamp();
        moduleIO.updateInputs(inputs);

        final int samples = inputs.odometryTimestampsSec.length;
        odometryPositions = new SwerveModulePosition[samples];
        for (int i = 0; i < samples; i++) {
            odometryPositions[i] = new SwerveModulePosition(
                    inputs.odometryDrivePositionsRots[i] * Modules.WHEEL_CIRCUMFERENCE_M,
                    Rotation2d.fromRotations(inputs.odometryTurnPositionRots[i])
            );
        }

        Logger.processInputs(logKey, inputs);

        Logger.recordOutput(logKey + "/CurrentState", getState());
        Logger.recordOutput(logKey + "/LastDesiredState", lastDesiredState);
        Logger.recordOutput(
                logKey + "/DriveDesiredVelocityRotsPerSec",
                computeDesiredDriverVelocity(
                        lastDesiredState,
                        Rotation2d.fromRotations(inputs.turnAbsolutePositionRots)
                )
        );

        Logger.recordOutput(
                logKey + "/TurnDesiredAbsolutePositionRots",
                computeDesiredTurnerRotations(lastDesiredState)
        );

        Logger.recordOutput(
                logKey + "/PeriodicIOPeriodMs",
                LogUtils.microsecondsToMilliseconds(Logger.getRealTimestamp() - modulePeriodicUpdateStart)
        );
    }

    /**
     * Get a {@link Rotation2d} of the current absolute turn position (computed from encoder rotations)
     * @return the absolute turn position as a {@link Rotation2d}
     * @see Rotation2d
     */
    public Rotation2d getAngle() {
        return Rotation2d.fromRotations(inputs.turnAbsolutePositionRots);
    }

    /**
     * Get the current relative drive wheel (mechanism) position in rotations
     * @return drive wheel position (rots)
     */
    public double getDrivePosition() {
        return inputs.drivePositionRots;
    }

    /**
     * Get the current drive wheel (mechanism) velocity in rotations/sec
     * @return drive wheel velocity (rps)
     */
    public double getDriveVelocity() {
        return inputs.driveVelocityRotsPerSec;
    }

    /**
     * Get the current module observed {@link SwerveModuleState} (velocity, angle)
     * Velocity is wheel linear velocity, angle is wheel absolute position
     * @return the module's current state as a {@link SwerveModuleState}
     * @see SwerveModuleState
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(
                getDriveVelocity() * Modules.WHEEL_CIRCUMFERENCE_M,
                getAngle()
        );
    }

    /**
     * Gets the current module observed {@link SwerveModulePosition} (position, angle)
     * Velocity is wheel linear position, angle is wheel absolute position
     * @return the module's current position as a {@link SwerveModulePosition}
     * @see SwerveModulePosition
     */
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
                getDrivePosition() * Modules.WHEEL_CIRCUMFERENCE_M,
                getAngle()
        );
    }

    public SwerveModulePosition[] getOdometryPositions() {
        return odometryPositions;
    }

    public double[] getOdometryTimestamps() {
        return inputs.odometryTimestampsSec;
    }

    /**
     * Characterizes the driving motor of the {@link SwerveModule} using
     * {@link SwerveModuleIO#setDriveCharacterizationVolts(double, double)}, while holding the turning at zero
     * @param volts the volts to apply to the drive motor
     */
    public void driveCharacterization(final double volts) {
        moduleIO.setDriveCharacterizationVolts(volts, 0);
    }

    /**
     * Computes the desired drive motor velocity given a desired {@link SwerveModuleState}
     * Characterizes the driving motor of the {@link SwerveModule} using
     * {@link SwerveModuleIO#setDriveCharacterizationVolts(double, double)}, while holding the turning at zero
     * @param driveVolts the volts to apply to the drive motor
     */
    public void driveVoltageCharacterization(final double driveVolts, final double turnPositionRots) {
        moduleIO.setDriveCharacterizationVolts(driveVolts, turnPositionRots);
    }

    /**
     * Characterizes the driving motor of the {@link SwerveModule} using
     * {@link SwerveModuleIO#setDriveCharacterizationAmps(double, double)}, while holding the turning at zero
     * @param driveTorqueCurrentAmps the torque current amps to apply to the drive motor
     */
    public void driveTorqueCurrentCharacterization(final double driveTorqueCurrentAmps, final double turnPositionRots) {
        moduleIO.setDriveCharacterizationAmps(driveTorqueCurrentAmps, turnPositionRots);
    }

    /**
     * Scales a {@link SwerveModuleState} by the cosine of the error between the {@link SwerveModuleState#angle} and
     * the measured angle (wheel rotation) by mutating its {@link SwerveModuleState#speedMetersPerSecond}.
     * <p> This should be called <b>AFTER</b> {@link SwerveModuleState#optimize(SwerveModuleState, Rotation2d)}</p>
     * @param state the {@link SwerveModuleState} to scale (this is mutated!)
     * @param wheelRotation the measured wheel {@link Rotation2d}
     */
    public static void scaleWithErrorCosine(final SwerveModuleState state, final Rotation2d wheelRotation) {
        // see https://github.com/wpilibsuite/allwpilib/issues/5749
        state.speedMetersPerSecond *= state.angle.minus(wheelRotation).getCos();
    }

    /**
     * Computes the desired drive motor velocity given a desired {@link SwerveModuleState}
     * i.e. the rotor velocity given wheel velocity (rps)
     * @param wantedState the wanted state of the module
     * @return the desired rotor velocity
     * @see SwerveModuleState
     */
    public double computeDesiredDriverVelocity(final SwerveModuleState wantedState, final Rotation2d wheelRotation) {
        SwerveModule.scaleWithErrorCosine(wantedState, wheelRotation);
        return wantedState.speedMetersPerSecond / Modules.WHEEL_CIRCUMFERENCE_M;
    }

    /**
     * Computes the desired turn motor velocity given a desired {@link SwerveModuleState}
     * i.e. the rotor position given wheel rotational position (rots)
     * @param wantedState the wanted state of the module
     * @return the desired rotor position
     * @see SwerveModuleState
     */
    public double computeDesiredTurnerRotations(final SwerveModuleState wantedState) {
        return wantedState.angle.getRotations();
    }

    /**
     * Sets the desired {@link SwerveModuleState} of the module
     * @param state the desired {@link SwerveModuleState}
     * @see SwerveModuleState
     */
    public void setDesiredState(final SwerveModuleState state) {
        final Rotation2d currentWheelRotation = getAngle();

        final SwerveModuleState wantedState = SwerveModuleState.optimize(state, currentWheelRotation);
        final double desiredDriverVelocity = computeDesiredDriverVelocity(wantedState, currentWheelRotation);
        final double desiredTurnerRotations = computeDesiredTurnerRotations(wantedState);

        this.lastDesiredState = wantedState;
        moduleIO.setInputs(desiredDriverVelocity, desiredTurnerRotations);
    }

    /**
     * Gets the last desired {@link SwerveModuleState} set in {@link SwerveModule#setDesiredState(SwerveModuleState)}
     * <p>
     * Note: this {@link SwerveModuleState} has been optimized and does not guarantee that it matches the last set state
     * @return the last desired {@link SwerveModuleState}
     */
    public SwerveModuleState getLastDesiredState() {
        return lastDesiredState;
    }

    /**
     * @see SwerveModuleIO#setNeutralMode(NeutralModeValue)
     */
    public void setNeutralMode(final NeutralModeValue neutralMode) {
        moduleIO.setNeutralMode(neutralMode);
    }
}
