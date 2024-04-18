package frc.robot.subsystems.climb;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoTorqueCurrentFOC;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.constants.HardwareConstants;

public class ClimbIOReal implements ClimbIO {
    private final HardwareConstants.ClimbConstants climbConstants;

    private final TalonFX rightArm;
    private final TalonFX leftArm;

    private final MotionMagicExpoTorqueCurrentFOC motionMagicExpoTorqueCurrentFOC;
    private final TorqueCurrentFOC torqueCurrentFOC;
    private final VoltageOut voltageOut;

    private final StatusSignal<Double> _rightArmPosition;
    private final StatusSignal<Double> _rightArmVelocity;
    private final StatusSignal<Double> _rightArmVoltage;
    private final StatusSignal<Double> _rightArmTorqueCurrent;
    private final StatusSignal<Double> _rightArmDeviceTemp;

    private final StatusSignal<Double> _leftArmPosition;
    private final StatusSignal<Double> _leftArmVelocity;
    private final StatusSignal<Double> _leftArmVoltage;
    private final StatusSignal<Double> _leftArmTorqueCurrent;
    private final StatusSignal<Double> _leftArmDeviceTemp;

    public ClimbIOReal(final HardwareConstants.ClimbConstants climbConstants) {
        this.climbConstants = climbConstants;

        this.rightArm = new TalonFX(climbConstants.rightArmMotorId(), climbConstants.CANBus());
        this.leftArm = new TalonFX(climbConstants.leftArmMotorId(), climbConstants.CANBus());

        this.motionMagicExpoTorqueCurrentFOC = new MotionMagicExpoTorqueCurrentFOC(0);
        this.torqueCurrentFOC = new TorqueCurrentFOC(0);
        this.voltageOut = new VoltageOut(0);

        this._rightArmPosition = rightArm.getPosition();
        this._rightArmVelocity = rightArm.getVelocity();
        this._rightArmVoltage = rightArm.getMotorVoltage();
        this._rightArmTorqueCurrent = rightArm.getTorqueCurrent();
        this._rightArmDeviceTemp = rightArm.getDeviceTemp();

        this._leftArmPosition = leftArm.getPosition();
        this._leftArmVelocity = leftArm.getVelocity();
        this._leftArmVoltage = leftArm.getMotorVoltage();
        this._leftArmTorqueCurrent = leftArm.getTorqueCurrent();
        this._leftArmDeviceTemp = leftArm.getDeviceTemp();
    }

    @Override
    public void updateInputs(final ClimbIO.ClimbIOInputs inputs) {
        BaseStatusSignal.refreshAll(
                _rightArmPosition,
                _rightArmVelocity,
                _rightArmVoltage,
                _rightArmTorqueCurrent,
                _rightArmDeviceTemp,
                _leftArmPosition,
                _leftArmVelocity,
                _leftArmVoltage,
                _leftArmTorqueCurrent,
                _leftArmDeviceTemp
        );

        inputs.rightPositionRots = _rightArmPosition.getValue();
        inputs.rightVelocityRotsPerSec = _rightArmVelocity.getValue();
        inputs.rightVoltage = _rightArmVoltage.getValue();
        inputs.rightTorqueCurrentAmps = _rightArmTorqueCurrent.getValue();
        inputs.rightTempCelsius = _rightArmDeviceTemp.getValue();

        inputs.leftPositionRots = _leftArmPosition.getValue();
        inputs.leftVelocityRotsPerSec = _leftArmVelocity.getValue();
        inputs.leftVoltage = _leftArmVoltage.getValue();
        inputs.leftTorqueCurrentAmps = _leftArmTorqueCurrent.getValue();
        inputs.leftTempCelsius = _leftArmDeviceTemp.getValue();
    }

    @Override
    public void config() {
        final TalonFXConfiguration rightArmConfig = new TalonFXConfiguration();
        // TODO: needs FF and tuned PID
        rightArmConfig.Slot0 = new Slot0Configs()
                .withKP(12)
                .withKA(0)
                .withKV(0)
                .withKS(0);
        rightArmConfig.TorqueCurrent.PeakForwardTorqueCurrent = 80;
        rightArmConfig.TorqueCurrent.PeakReverseTorqueCurrent = -80;
        rightArmConfig.CurrentLimits.StatorCurrentLimit = 60;
        rightArmConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        rightArmConfig.CurrentLimits.SupplyCurrentLimit = 40;
        rightArmConfig.CurrentLimits.SupplyCurrentThreshold = 55;
        rightArmConfig.CurrentLimits.SupplyTimeThreshold = 1.5;
        rightArmConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        rightArmConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        rightArmConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        rightArmConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        rightArmConfig.Feedback.SensorToMechanismRatio = climbConstants.climbGearing();
        rightArm.getConfigurator().apply(rightArmConfig);

        final TalonFXConfiguration leftArmConfig = new TalonFXConfiguration();
        // TODO: needs FF and tuned PID
        leftArmConfig.Slot0 = new Slot0Configs()
                .withKP(12)
                .withKA(0)
                .withKV(0)
                .withKS(0);
        leftArmConfig.TorqueCurrent.PeakForwardTorqueCurrent = 80;
        leftArmConfig.TorqueCurrent.PeakReverseTorqueCurrent = -80;
        leftArmConfig.CurrentLimits.StatorCurrentLimit = 60;
        leftArmConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        leftArmConfig.CurrentLimits.SupplyCurrentLimit = 40;
        leftArmConfig.CurrentLimits.SupplyCurrentThreshold = 55;
        leftArmConfig.CurrentLimits.SupplyTimeThreshold = 1.5;
        leftArmConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        leftArmConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        leftArmConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        leftArmConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        leftArmConfig.Feedback.SensorToMechanismRatio = climbConstants.climbGearing();
        leftArm.getConfigurator().apply(leftArmConfig);

        BaseStatusSignal.setUpdateFrequencyForAll(
                100,
                _rightArmPosition,
                _rightArmVelocity,
                _rightArmVoltage,
                _rightArmTorqueCurrent,
                _leftArmPosition,
                _leftArmVelocity,
                _leftArmVoltage,
                _leftArmTorqueCurrent
        );

        BaseStatusSignal.setUpdateFrequencyForAll(
                4,
                _rightArmDeviceTemp,
                _leftArmDeviceTemp
        );
        ParentDevice.optimizeBusUtilizationForAll(
                rightArm,
                leftArm
        );

        rightArm.setPosition(0);
        leftArm.setPosition(0);
    }

    @Override
    public void toPosition(
            final double rightArmPosition,
            final double leftArmPosition
    ) {
        rightArm.setControl(motionMagicExpoTorqueCurrentFOC.withPosition(rightArmPosition));
        leftArm.setControl(motionMagicExpoTorqueCurrentFOC.withPosition(leftArmPosition));
    }

    @Override
    public void toTorqueCurrent(
            final double rightArmTorqueCurrentAmps,
            final double leftArmTorqueCurrentAmps
    ) {
        rightArm.setControl(torqueCurrentFOC.withOutput(rightArmTorqueCurrentAmps));
        leftArm.setControl(torqueCurrentFOC.withOutput(leftArmTorqueCurrentAmps));
    }

    @Override
    public void toVoltage(double rightArmVolts, double leftArmVolts) {
        rightArm.setControl(voltageOut.withOutput(rightArmVolts));
        leftArm.setControl(voltageOut.withOutput(leftArmVolts));
    }
}
