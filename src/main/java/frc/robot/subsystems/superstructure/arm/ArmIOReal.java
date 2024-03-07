package frc.robot.subsystems.superstructure.arm;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.MotionMagicExpoTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.constants.HardwareConstants;
import frc.robot.utils.ctre.Phoenix6Utils;

public class ArmIOReal implements ArmIO {
    private final HardwareConstants.ArmConstants armConstants;

    private final TalonFX leftPivotMotor;
    private final TalonFX rightPivotMotor;

//    private final MotionMagicExpoTorqueCurrentFOC motionMagicExpoTorqueCurrentFOC;
    private final MotionMagicExpoVoltage motionMagicExpoVoltage;
    private final TorqueCurrentFOC torqueCurrentFOC;
    private final VoltageOut voltageOut;

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

    public ArmIOReal(final HardwareConstants.ArmConstants armConstants) {
        this.armConstants = armConstants;

        this.leftPivotMotor = new TalonFX(armConstants.leftPivotMotorId(), armConstants.CANBus());
        this.rightPivotMotor = new TalonFX(armConstants.rightPivotMotorId(), armConstants.CANBus());

//        this.motionMagicExpoTorqueCurrentFOC = new MotionMagicExpoTorqueCurrentFOC(0);
        this.motionMagicExpoVoltage = new MotionMagicExpoVoltage(0);
        this.torqueCurrentFOC = new TorqueCurrentFOC(0);
        this.voltageOut = new VoltageOut(0);

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
    }

    @SuppressWarnings("DuplicatedCode")
    @Override
    public void config() {
        final TalonFXConfiguration leftTalonFXConfiguration = new TalonFXConfiguration();
        final InvertedValue leftTalonFXInverted = InvertedValue.Clockwise_Positive;
        leftTalonFXConfiguration.Slot0 = new Slot0Configs()
//                .withKS(0)
//                .withKG(0.11)
//                .withGravityType(GravityTypeValue.Arm_Cosine)
//                .withKV(13.97)
//                .withKA(0.015)
//                .withKP(50); // TODO: tune Kp

//                .withKS(0.16803)
//                .withKG(0.16766)
//                .withGravityType(GravityTypeValue.Arm_Cosine)
//                .withKV(13.623)
//                .withKA(0.2573 / 2)
                .withKP(30);
//                .withKD(3.5442);
        leftTalonFXConfiguration.MotionMagic.MotionMagicCruiseVelocity = 0;
        leftTalonFXConfiguration.MotionMagic.MotionMagicExpo_kV = 13.623;
        leftTalonFXConfiguration.MotionMagic.MotionMagicExpo_kA = 0.015;
        leftTalonFXConfiguration.TorqueCurrent.PeakForwardTorqueCurrent = 80;
        leftTalonFXConfiguration.TorqueCurrent.PeakReverseTorqueCurrent = -80;
        leftTalonFXConfiguration.CurrentLimits.StatorCurrentLimit = 60;
        leftTalonFXConfiguration.CurrentLimits.StatorCurrentLimitEnable = true;
        leftTalonFXConfiguration.Feedback.SensorToMechanismRatio = armConstants.pivotGearing();
        leftTalonFXConfiguration.MotorOutput.Inverted = leftTalonFXInverted;
        leftTalonFXConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        leftPivotMotor.getConfigurator().apply(leftTalonFXConfiguration);

        final TalonFXConfiguration rightTalonFXConfiguration = new TalonFXConfiguration();
        final InvertedValue rightTalonFXInverted = InvertedValue.CounterClockwise_Positive;
        rightTalonFXConfiguration.Slot0 = new Slot0Configs()
//                .withKS(0)
//                .withKG(0.11)
//                .withGravityType(GravityTypeValue.Arm_Cosine)
//                .withKV(13.97)
//                .withKA(0.015)
//                .withKP(50); // TODO: tune Kp

//                .withKS(0.16803)
//                .withKG(0.16766)
//                .withGravityType(GravityTypeValue.Arm_Cosine)
//                .withKV(13.623)
//                .withKA(0.2573 / 2)
                .withKP(30);
//                .withKD(3.5442);
        rightTalonFXConfiguration.MotionMagic.MotionMagicCruiseVelocity = 0;
        rightTalonFXConfiguration.MotionMagic.MotionMagicExpo_kV = 13.97;
        rightTalonFXConfiguration.MotionMagic.MotionMagicExpo_kA = 0.015;
        rightTalonFXConfiguration.TorqueCurrent.PeakForwardTorqueCurrent = 80;
        rightTalonFXConfiguration.TorqueCurrent.PeakReverseTorqueCurrent = -80;
        rightTalonFXConfiguration.CurrentLimits.StatorCurrentLimit = 60;
        rightTalonFXConfiguration.CurrentLimits.StatorCurrentLimitEnable = true;
        rightTalonFXConfiguration.Feedback.SensorToMechanismRatio = armConstants.pivotGearing();
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
                _rightTorqueCurrent
        );
        BaseStatusSignal.setUpdateFrequencyForAll(
                4,
                _leftDeviceTemp,
                _rightDeviceTemp
        );
        ParentDevice.optimizeBusUtilizationForAll(
                leftPivotMotor,
                rightPivotMotor
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
                _rightDeviceTemp
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
    }

    @Override
    public void toPivotPosition(final double pivotPositionRots) {
        leftPivotMotor.setControl(motionMagicExpoVoltage.withPosition(pivotPositionRots));
        rightPivotMotor.setControl(motionMagicExpoVoltage.withPosition(pivotPositionRots));
    }

    @Override
    public void toPivotVoltage(final double pivotVolts) {
        leftPivotMotor.setControl(voltageOut.withOutput(pivotVolts));
        rightPivotMotor.setControl(voltageOut.withOutput(pivotVolts));
    }

    @Override
    public void configureSoftLimits(
            final Arm.PositionSetpoint pivotSoftLowerLimit,
            final Arm.PositionSetpoint pivotSoftUpperLimit
    ) {
        final TalonFXConfigurator leftConfigurator = leftPivotMotor.getConfigurator();
        final TalonFXConfiguration leftConfiguration = new TalonFXConfiguration();
        Phoenix6Utils.reportIfNotOk(leftPivotMotor, leftConfigurator.refresh(leftConfiguration));

        leftConfiguration.SoftwareLimitSwitch.ReverseSoftLimitThreshold = pivotSoftLowerLimit.pivotPositionRots;
        leftConfiguration.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        leftConfiguration.SoftwareLimitSwitch.ForwardSoftLimitThreshold = pivotSoftUpperLimit.pivotPositionRots;
        leftConfiguration.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;

        Phoenix6Utils.reportIfNotOk(leftPivotMotor, leftConfigurator.apply(leftConfiguration));

        final TalonFXConfigurator rightConfigurator = rightPivotMotor.getConfigurator();
        final TalonFXConfiguration rightConfiguration = new TalonFXConfiguration();
        Phoenix6Utils.reportIfNotOk(rightPivotMotor, rightConfigurator.refresh(rightConfiguration));

        rightConfiguration.SoftwareLimitSwitch.ReverseSoftLimitThreshold = pivotSoftLowerLimit.pivotPositionRots;
        rightConfiguration.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        rightConfiguration.SoftwareLimitSwitch.ForwardSoftLimitThreshold = pivotSoftUpperLimit.pivotPositionRots;
        rightConfiguration.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;

        Phoenix6Utils.reportIfNotOk(rightPivotMotor, rightConfigurator.apply(rightConfiguration));
    }

    @Override
    public void toPivotTorqueCurrent(double pivotTorqueCurrentAmps) {
        leftPivotMotor.setControl(torqueCurrentFOC.withOutput(pivotTorqueCurrentAmps));
        rightPivotMotor.setControl(torqueCurrentFOC.withOutput(pivotTorqueCurrentAmps));
    }
}
