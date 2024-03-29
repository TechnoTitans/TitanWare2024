package frc.robot.subsystems.superstructure.arm;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.constants.HardwareConstants;
import frc.robot.utils.ctre.Phoenix6Utils;

public class ArmIOReal implements ArmIO {
    private final HardwareConstants.ArmConstants armConstants;

    private final TalonFX leftPivotMotor;
    private final TalonFX rightPivotMotor;
    private final CANcoder pivotCANCoder;

    // TODO: use MotionMagicExpoTorqueCurrentFOC
//    private final MotionMagicExpoTorqueCurrentFOC motionMagicExpoTorqueCurrentFOC;
    private final MotionMagicExpoVoltage motionMagicExpoVoltage;
    private final TorqueCurrentFOC torqueCurrentFOC;
    private final VoltageOut voltageOut;

    private final Follower leftPivotFollower;

    // Cached StatusSignals
    private final StatusSignal<Double> _leftPosition;
    private final StatusSignal<Double> _leftVelocity;
    private final StatusSignal<Double> _leftVoltage;
    private final StatusSignal<Double> _leftTorqueCurrent;
    private final StatusSignal<Double> _leftDeviceTemp;
    private final StatusSignal<Double> _rightPosition;
    private final StatusSignal<Double> _rightVelocity;
    private final StatusSignal<Double> _rightVoltage;
    private final StatusSignal<Double> _rightTorqueCurrent;
    private final StatusSignal<Double> _rightDeviceTemp;
    private final StatusSignal<Double> _encoderPosition;
    private final StatusSignal<Double> _encoderVelocity;

    public ArmIOReal(final HardwareConstants.ArmConstants armConstants) {
        this.armConstants = armConstants;

        this.leftPivotMotor = new TalonFX(armConstants.leftPivotMotorId(), armConstants.CANBus());
        this.rightPivotMotor = new TalonFX(armConstants.rightPivotMotorId(), armConstants.CANBus());
        this.pivotCANCoder = new CANcoder(armConstants.pivotCANCoderId(), armConstants.CANBus());

//        this.motionMagicExpoTorqueCurrentFOC = new MotionMagicExpoTorqueCurrentFOC(0);
        this.motionMagicExpoVoltage = new MotionMagicExpoVoltage(0);
        this.torqueCurrentFOC = new TorqueCurrentFOC(0);
        this.voltageOut = new VoltageOut(0);

        this.leftPivotFollower = new Follower(leftPivotMotor.getDeviceID(), true);

        this._leftPosition = leftPivotMotor.getPosition();
        this._leftVelocity = leftPivotMotor.getVelocity();
        this._leftVoltage = leftPivotMotor.getMotorVoltage();
        this._leftTorqueCurrent = leftPivotMotor.getTorqueCurrent();
        this._leftDeviceTemp = leftPivotMotor.getDeviceTemp();
        this._rightPosition = rightPivotMotor.getPosition();
        this._rightVelocity = rightPivotMotor.getVelocity();
        this._rightVoltage = rightPivotMotor.getMotorVoltage();
        this._rightTorqueCurrent = rightPivotMotor.getTorqueCurrent();
        this._rightDeviceTemp = rightPivotMotor.getDeviceTemp();
        this._encoderPosition = pivotCANCoder.getPosition();
        this._encoderVelocity = pivotCANCoder.getVelocity();
    }

    @SuppressWarnings("DuplicatedCode")
    @Override
    public void config() {
        final CANcoderConfiguration pivotCANCoderConfiguration = new CANcoderConfiguration();
        pivotCANCoderConfiguration.MagnetSensor.MagnetOffset = armConstants.pivotCANCoderOffset();
        pivotCANCoder.getConfigurator().apply(pivotCANCoderConfiguration);

        final TalonFXConfiguration leftTalonFXConfiguration = new TalonFXConfiguration();
        final InvertedValue leftTalonFXInverted = InvertedValue.Clockwise_Positive;
        leftTalonFXConfiguration.Slot0 = new Slot0Configs()
//                .withKS(0.011965)
                .withKS(0.15)
                .withKG(0.22)
                .withGravityType(GravityTypeValue.Arm_Cosine)
                .withKV(14.053)
                .withKA(0.17176 * 0.5)
                .withKP(16.658); // TODO: tune Kp
        leftTalonFXConfiguration.MotionMagic.MotionMagicCruiseVelocity = 0;
        leftTalonFXConfiguration.MotionMagic.MotionMagicExpo_kV = 14.053;
//        leftTalonFXConfiguration.MotionMagic.MotionMagicExpo_kA = 0.17176;
        leftTalonFXConfiguration.MotionMagic.MotionMagicExpo_kA = 1;
        leftTalonFXConfiguration.TorqueCurrent.PeakForwardTorqueCurrent = 80;
        leftTalonFXConfiguration.TorqueCurrent.PeakReverseTorqueCurrent = -80;
        leftTalonFXConfiguration.CurrentLimits.StatorCurrentLimit = 60;
        leftTalonFXConfiguration.CurrentLimits.StatorCurrentLimitEnable = true;
        leftTalonFXConfiguration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        leftTalonFXConfiguration.Feedback.FeedbackRemoteSensorID = pivotCANCoder.getDeviceID();
        leftTalonFXConfiguration.Feedback.RotorToSensorRatio = armConstants.pivotGearing();
        leftTalonFXConfiguration.MotorOutput.Inverted = leftTalonFXInverted;
        leftTalonFXConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        leftPivotMotor.getConfigurator().apply(leftTalonFXConfiguration);

        final TalonFXConfiguration rightTalonFXConfiguration = new TalonFXConfiguration();
        final InvertedValue rightTalonFXInverted = InvertedValue.CounterClockwise_Positive;
        rightTalonFXConfiguration.Slot0 = new Slot0Configs()
//                .withKS(0.011965)
                .withKS(0.15)
                .withKG(0.22)
                .withGravityType(GravityTypeValue.Arm_Cosine)
                .withKV(14.053)
                .withKA(0.17176 * 0.5)
                .withKP(16.658); // TODO: tune Kp
        rightTalonFXConfiguration.MotionMagic.MotionMagicCruiseVelocity = 0;
        rightTalonFXConfiguration.MotionMagic.MotionMagicExpo_kV = 14.053;
//        rightTalonFXConfiguration.MotionMagic.MotionMagicExpo_kA = 0.17176;
        rightTalonFXConfiguration.MotionMagic.MotionMagicExpo_kA = 1;
        rightTalonFXConfiguration.TorqueCurrent.PeakForwardTorqueCurrent = 80;
        rightTalonFXConfiguration.TorqueCurrent.PeakReverseTorqueCurrent = -80;
        rightTalonFXConfiguration.CurrentLimits.StatorCurrentLimit = 60;
        rightTalonFXConfiguration.CurrentLimits.StatorCurrentLimitEnable = true;
        rightTalonFXConfiguration.Feedback.RotorToSensorRatio = armConstants.pivotGearing();
        rightTalonFXConfiguration.MotorOutput.Inverted = rightTalonFXInverted;
        rightTalonFXConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        rightPivotMotor.getConfigurator().apply(rightTalonFXConfiguration);

        BaseStatusSignal.setUpdateFrequencyForAll(
                100,
                _leftPosition,
                _leftVelocity,
                _leftVoltage,
                _leftTorqueCurrent,
                _rightPosition,
                _rightVelocity,
                _rightVoltage,
                _rightTorqueCurrent,
                _encoderPosition,
                _encoderVelocity
        );
        BaseStatusSignal.setUpdateFrequencyForAll(
                4,
                _leftDeviceTemp,
                _rightDeviceTemp
        );
        ParentDevice.optimizeBusUtilizationForAll(
                leftPivotMotor,
                rightPivotMotor,
                pivotCANCoder
        );
    }

    @SuppressWarnings("DuplicatedCode")
    @Override
    public void updateInputs(final ArmIO.ArmIOInputs inputs) {
        BaseStatusSignal.refreshAll(
                _leftPosition,
                _leftVelocity,
                _leftVoltage,
                _leftTorqueCurrent,
                _leftDeviceTemp,
                _rightPosition,
                _rightVelocity,
                _rightVoltage,
                _rightTorqueCurrent,
                _rightDeviceTemp,
                _encoderPosition,
                _encoderVelocity
        );

        inputs.leftPivotPositionRots = _leftPosition.getValue();
        inputs.leftPivotVelocityRotsPerSec = _leftVelocity.getValue();
        inputs.leftPivotVoltageVolts = _leftVoltage.getValue();
        inputs.leftPivotTorqueCurrentAmps = _leftTorqueCurrent.getValue();
        inputs.leftPivotTempCelsius = _leftDeviceTemp.getValue();

        inputs.rightPivotPositionRots = _rightPosition.getValue();
        inputs.rightPivotVelocityRotsPerSec = _rightVelocity.getValue();
        inputs.rightPivotVoltageVolts = _rightVoltage.getValue();
        inputs.rightPivotTorqueCurrentAmps = _rightTorqueCurrent.getValue();
        inputs.rightPivotTempCelsius = _rightDeviceTemp.getValue();

        inputs.pivotEncoderPositionRots = _encoderPosition.getValue();
        inputs.pivotEncoderVelocityRotsPerSec = _encoderVelocity.getValue();
    }

    @Override
    public void configureSoftLimits(
            final Arm.PositionSetpoint pivotSoftLowerLimit,
            final Arm.PositionSetpoint pivotSoftUpperLimit
    ) {
        Phoenix6Utils.configureTalonFXSoftLimits(
                leftPivotMotor,
                pivotSoftLowerLimit.pivotPositionRots,
                pivotSoftUpperLimit.pivotPositionRots
        );

        Phoenix6Utils.configureTalonFXSoftLimits(
                rightPivotMotor,
                pivotSoftLowerLimit.pivotPositionRots,
                pivotSoftUpperLimit.pivotPositionRots
        );
    }

    @Override
    public void setPivotPosition(final double pivotPositionRots) {
        // TODO: this should probably retry it if it fails in addition to just reporting it
        Phoenix6Utils.reportIfNotOk(leftPivotMotor, leftPivotMotor.setPosition(pivotPositionRots));
        Phoenix6Utils.reportIfNotOk(rightPivotMotor, rightPivotMotor.setPosition(pivotPositionRots));
    }

    @Override
    public void toPivotPosition(final double pivotPositionRots) {
//        leftPivotMotor.setControl(motionMagicExpoTorqueCurrentFOC.withPosition(pivotPositionRots));
        leftPivotMotor.setControl(motionMagicExpoVoltage.withPosition(pivotPositionRots));
        rightPivotMotor.setControl(leftPivotFollower);
    }

    @Override
    public void toPivotVoltage(final double pivotVolts) {
        leftPivotMotor.setControl(voltageOut.withOutput(pivotVolts));
        rightPivotMotor.setControl(leftPivotFollower);
    }

    @Override
    public void toPivotTorqueCurrent(double pivotTorqueCurrentAmps) {
        leftPivotMotor.setControl(torqueCurrentFOC.withOutput(pivotTorqueCurrentAmps));
        rightPivotMotor.setControl(leftPivotFollower);
    }
}
