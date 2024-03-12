package frc.robot.subsystems.drive;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.util.DoubleCircularBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.constants.Constants.Swerve.Modules;
import frc.robot.constants.HardwareConstants;
import frc.robot.utils.ctre.Phoenix6Utils;

public class SwerveModuleIOTalonFX implements SwerveModuleIO {
    private final TalonFX driveMotor;
    private final TalonFX turnMotor;
    private final CANcoder turnEncoder;
    private final double magnetOffset;

    private final TalonFXConfiguration driveTalonFXConfiguration = new TalonFXConfiguration();
    private final TalonFXConfiguration turnTalonFXConfiguration = new TalonFXConfiguration();

    private final VelocityTorqueCurrentFOC velocityTorqueCurrentFOC;
    private final VoltageOut voltageOut;
    private final TorqueCurrentFOC torqueCurrentFOC;

    private final PositionVoltage positionVoltage;

    private final OdometryThreadRunner odometryThreadRunner;
    // Cached StatusSignals
    private final StatusSignal<Double> _drivePosition;
    private final StatusSignal<Double> _driveVelocity;
    private final StatusSignal<Double> _driveTorqueCurrent;
    private final StatusSignal<Double> _driveStatorCurrent;
    private final StatusSignal<Double> _driveDeviceTemp;
    private final StatusSignal<Double> _turnPosition;
    private final StatusSignal<Double> _turnVelocity;
    private final StatusSignal<Double> _turnTorqueCurrent;
    private final StatusSignal<Double> _turnStatorCurrent;
    private final StatusSignal<Double> _turnDeviceTemp;

    // Odometry StatusSignal update buffers
    private final DoubleCircularBuffer timestampBuffer;
    private final DoubleCircularBuffer drivePositionSignalBuffer;
    private final DoubleCircularBuffer turnPositionSignalBuffer;

    public SwerveModuleIOTalonFX(
            final HardwareConstants.SwerveModuleConstants constants,
            final OdometryThreadRunner odometryThreadRunner
    ) {
        this.driveMotor = new TalonFX(constants.driveMotorId(), constants.moduleCANBus());
        this.turnMotor = new TalonFX(constants.turnMotorId(), constants.moduleCANBus());

        this.turnEncoder = new CANcoder(constants.turnEncoderId(), constants.moduleCANBus());
        this.magnetOffset = constants.turnEncoderOffset();

        this.velocityTorqueCurrentFOC = new VelocityTorqueCurrentFOC(0);
        this.torqueCurrentFOC = new TorqueCurrentFOC(0);
        this.positionVoltage = new PositionVoltage(0);
        this.voltageOut = new VoltageOut(0);

        this.odometryThreadRunner = odometryThreadRunner;
        this.odometryThreadRunner.registerControlRequest(driveMotor, velocityTorqueCurrentFOC, driveMotor::setControl);
        this.odometryThreadRunner.registerControlRequest(turnMotor, positionVoltage, turnMotor::setControl);

        this._drivePosition = driveMotor.getPosition();
        this._driveVelocity = driveMotor.getVelocity();
        this._driveTorqueCurrent = driveMotor.getTorqueCurrent();
        this._driveStatorCurrent = driveMotor.getStatorCurrent();
        this._driveDeviceTemp = driveMotor.getDeviceTemp();
        this._turnPosition = turnEncoder.getAbsolutePosition();
        this._turnVelocity = turnEncoder.getVelocity();
        this._turnTorqueCurrent = turnMotor.getTorqueCurrent();
        this._turnStatorCurrent = turnMotor.getStatorCurrent();
        this._turnDeviceTemp = turnMotor.getDeviceTemp();

        this.timestampBuffer = odometryThreadRunner.makeTimestampBuffer();
        this.drivePositionSignalBuffer = odometryThreadRunner.registerSignal(driveMotor, _drivePosition);
        this.turnPositionSignalBuffer = odometryThreadRunner.registerSignal(turnMotor, _turnPosition);
    }

    @SuppressWarnings("DuplicatedCode")
    @Override
    public void config() {
        // TODO: check StatusCode of some/most of these blocking config calls... maybe retry if failed?
        final CANcoderConfiguration canCoderConfiguration = new CANcoderConfiguration();
        canCoderConfiguration.MagnetSensor.MagnetOffset = -magnetOffset;
        canCoderConfiguration.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
        turnEncoder.getConfigurator().apply(canCoderConfiguration);

        // TODO: check these gains, or just re-tune
        driveTalonFXConfiguration.Slot0 = new Slot0Configs()
                .withKS(5.8432)
                .withKV(0)
                .withKA(2.4446)
                .withKP(23.506);
        driveTalonFXConfiguration.TorqueCurrent.PeakForwardTorqueCurrent = Modules.SLIP_CURRENT_A;
        driveTalonFXConfiguration.TorqueCurrent.PeakReverseTorqueCurrent = -Modules.SLIP_CURRENT_A;
        driveTalonFXConfiguration.CurrentLimits.StatorCurrentLimit = Modules.SLIP_CURRENT_A;
        driveTalonFXConfiguration.CurrentLimits.StatorCurrentLimitEnable = true;
        driveTalonFXConfiguration.CurrentLimits.SupplyCurrentLimit = 40;
        driveTalonFXConfiguration.CurrentLimits.SupplyCurrentThreshold = 50;
        driveTalonFXConfiguration.CurrentLimits.SupplyTimeThreshold = 0.5;
        driveTalonFXConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;
        driveTalonFXConfiguration.ClosedLoopRamps.TorqueClosedLoopRampPeriod = 0.2;
        driveTalonFXConfiguration.Feedback.SensorToMechanismRatio = Modules.DRIVER_GEAR_RATIO;
        driveTalonFXConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        driveTalonFXConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        driveMotor.getConfigurator().apply(driveTalonFXConfiguration);

        turnTalonFXConfiguration.Slot0 = new Slot0Configs()
                .withKP(30)
                .withKS(0.5);
        turnTalonFXConfiguration.TorqueCurrent.PeakForwardTorqueCurrent = 40;
        turnTalonFXConfiguration.TorqueCurrent.PeakReverseTorqueCurrent = -40;
        turnTalonFXConfiguration.CurrentLimits.StatorCurrentLimit = 60;
        turnTalonFXConfiguration.CurrentLimits.StatorCurrentLimitEnable = true;
        turnTalonFXConfiguration.CurrentLimits.SupplyCurrentLimit = 40;
        turnTalonFXConfiguration.CurrentLimits.SupplyCurrentThreshold = 60;
        turnTalonFXConfiguration.CurrentLimits.SupplyTimeThreshold = 1.5;
        turnTalonFXConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;
        turnTalonFXConfiguration.Feedback.FeedbackRemoteSensorID = turnEncoder.getDeviceID();
        turnTalonFXConfiguration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        turnTalonFXConfiguration.Feedback.RotorToSensorRatio = Modules.TURNER_GEAR_RATIO;
        turnTalonFXConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        turnTalonFXConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        turnMotor.getConfigurator().apply(turnTalonFXConfiguration);

        velocityTorqueCurrentFOC.UpdateFreqHz = 0;
        positionVoltage.UpdateFreqHz = 0;

        BaseStatusSignal.setUpdateFrequencyForAll(
                100,
                _driveVelocity,
                _driveTorqueCurrent,
                _driveStatorCurrent,
                _driveDeviceTemp,
                _turnVelocity,
                _turnTorqueCurrent,
                _turnStatorCurrent,
                _turnDeviceTemp
        );
        ParentDevice.optimizeBusUtilizationForAll(driveMotor, turnMotor);
    }

    @SuppressWarnings("DuplicatedCode")
    @Override
    public void updateInputs(final SwerveModuleIOInputs inputs) {
        BaseStatusSignal.refreshAll(
                _drivePosition,
                _driveVelocity,
                _driveTorqueCurrent,
                _driveStatorCurrent,
                _driveDeviceTemp,
                _turnPosition,
                _turnVelocity,
                _turnTorqueCurrent,
                _turnStatorCurrent,
                _turnDeviceTemp
        );

        inputs.drivePositionRots = getDrivePosition();
        inputs.driveVelocityRotsPerSec = _driveVelocity.getValue();
        inputs.driveTorqueCurrentAmps = _driveTorqueCurrent.getValue();
        inputs.driveStatorCurrentAmps = _driveStatorCurrent.getValue();
        inputs.driveTempCelsius = _driveDeviceTemp.getValue();

        inputs.turnAbsolutePositionRots = getRawAngle();
        inputs.turnVelocityRotsPerSec = _turnVelocity.getValue();
        inputs.turnTorqueCurrentAmps = _turnTorqueCurrent.getValue();
        inputs.turnStatorCurrentAmps = _turnStatorCurrent.getValue();
        inputs.turnTempCelsius = _turnDeviceTemp.getValue();

        inputs.odometryTimestampsSec = OdometryThreadRunner.writeBufferToArray(timestampBuffer);
        timestampBuffer.clear();

        inputs.odometryDrivePositionsRots = OdometryThreadRunner.writeBufferToArray(drivePositionSignalBuffer);
        drivePositionSignalBuffer.clear();

        inputs.odometryTurnPositionRots = OdometryThreadRunner.writeBufferToArray(turnPositionSignalBuffer);
        turnPositionSignalBuffer.clear();
    }

    /**
     * Get the measured wheel angle (mechanism) angle of the {@link SwerveModuleIO}, in raw units (rotations)
     * @return the measured wheel (turner) angle, in rotations
     */
    private double getRawAngle() {
        return Phoenix6Utils.latencyCompensateIfSignalIsGood(_turnPosition, _turnVelocity);
    }

    /**
     * Get the measured drive wheel (mechanism) position of the {@link SwerveModuleIO}, in raw units (rotations)
     * @return the measured drive wheel position, in rotations
     */
    public double getDrivePosition() {
        final double driveWheelPosition = Phoenix6Utils.latencyCompensateIfSignalIsGood(_drivePosition, _driveVelocity);
        final double turnPosition = Phoenix6Utils.latencyCompensateIfSignalIsGood(_turnPosition, _turnVelocity);
        final double driveBackOutWheelRotations = (
                (turnPosition * Modules.COUPLING_GEAR_RATIO)
                        / Modules.DRIVER_GEAR_RATIO
        );

        return driveWheelPosition - driveBackOutWheelRotations;
    }

    @Override
    public void setInputs(final double desiredDriverVelocity, final double desiredTurnerRotations) {
        final double driveVelocityBackOut = (
                (_turnVelocity.getValue() * Modules.COUPLING_GEAR_RATIO)
                        / Modules.DRIVER_GEAR_RATIO
        );
        final double backedOutDriveVelocity = desiredDriverVelocity + driveVelocityBackOut;

        odometryThreadRunner.updateControlRequest(driveMotor, velocityTorqueCurrentFOC);
        driveMotor.setControl(velocityTorqueCurrentFOC
                .withVelocity(backedOutDriveVelocity)
                .withOverrideCoastDurNeutral(true)
        );
        turnMotor.setControl(positionVoltage.withPosition(desiredTurnerRotations));
    }

    @Override
    public void setDriveCharacterizationVolts(double driveVolts, double desiredTurnerRotations) {
        odometryThreadRunner.updateControlRequest(driveMotor, voltageOut);
        driveMotor.setControl(voltageOut.withOutput(driveVolts));
        turnMotor.setControl(positionVoltage.withPosition(desiredTurnerRotations));
    }

    @Override
    public void setDriveCharacterizationAmps(double driveTorqueCurrentAmps, double desiredTurnerRotations) {
        odometryThreadRunner.updateControlRequest(driveMotor, torqueCurrentFOC);
        driveMotor.setControl(torqueCurrentFOC.withOutput(driveTorqueCurrentAmps));
        turnMotor.setControl(positionVoltage.withPosition(desiredTurnerRotations));
    }

    @Override
    public void setNeutralMode(final NeutralModeValue neutralMode) {
        final StatusCode refreshCode = driveMotor.getConfigurator().refresh(turnTalonFXConfiguration);
        if (!refreshCode.isOK()) {
            DriverStation.reportWarning(
                    String.format(
                            "Failed to set NeutralMode on TalonFX %s (%s)",
                            driveMotor.getDeviceID(),
                            driveMotor.getNetwork()
                    ), false
            );
            return;
        }

        turnTalonFXConfiguration.MotorOutput.NeutralMode = neutralMode;
        driveMotor.getConfigurator().apply(turnTalonFXConfiguration);
    }
}
