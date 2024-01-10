package frc.robot.subsystems.intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.*;
import frc.robot.constants.HardwareConstants;

public class IntakeIOTalonFXSim implements IntakeIO {
    private final TalonFX rollerMotor;
    private final TalonFX deployMotor;

    private final VelocityVoltage rollerVelocityVoltage;
    private final MotionMagicExpoTorqueCurrentFOC deployMotionMagicExpoTorqueCurrentFOC;

    private final StatusSignal<Double> _rollerMotorPosition;
    private final StatusSignal<Double> _rollerMotorVelocity;
    private final StatusSignal<Double> _rollerMotorTorqueCurrent;
    private final StatusSignal<Double> _rollerMotorDeviceTemp;
    private final StatusSignal<Double> _deployMotorPosition;
    private final StatusSignal<Double> _deployMotorVelocity;
    private final StatusSignal<Double> _deployMotorTorqueCurrent;
    private final StatusSignal<Double> _deployMotorDeviceTemp;
    private final StatusSignal<ReverseLimitValue> _deployMotorLimitSwitch;

    public IntakeIOTalonFXSim(final HardwareConstants.IntakeConstants intakeConstants) {
        this.rollerMotor = new TalonFX(intakeConstants.rollerMotorId(), intakeConstants.CANBus());
        this.deployMotor = new TalonFX(intakeConstants.deployMotorId(), intakeConstants.CANBus());

        this.rollerVelocityVoltage = new VelocityVoltage(0);
        this.deployMotionMagicExpoTorqueCurrentFOC = new MotionMagicExpoTorqueCurrentFOC(0);

        this._rollerMotorPosition = rollerMotor.getPosition();
        this._rollerMotorVelocity = rollerMotor.getVelocity();
        this._rollerMotorTorqueCurrent = rollerMotor.getTorqueCurrent();
        this._rollerMotorDeviceTemp = rollerMotor.getDeviceTemp();
        this._deployMotorPosition = deployMotor.getPosition();
        this._deployMotorVelocity = deployMotor.getVelocity();
        this._deployMotorTorqueCurrent = deployMotor.getTorqueCurrent();
        this._deployMotorDeviceTemp = deployMotor.getDeviceTemp();
        this._deployMotorLimitSwitch = deployMotor.getReverseLimit();
    }

    @Override
    public void periodic() {
        BaseStatusSignal.refreshAll(
                _rollerMotorPosition,
                _rollerMotorVelocity,
                _rollerMotorTorqueCurrent,
                _rollerMotorDeviceTemp,
                _deployMotorPosition,
                _deployMotorVelocity,
                _deployMotorTorqueCurrent,
                _deployMotorDeviceTemp,
                _deployMotorLimitSwitch
        );
    }

    @Override
    public void updateInputs(final IntakeIOInputs inputs) {
        inputs.rollerMotorPositionRots = _rollerMotorPosition.getValue();
        inputs.rollerMotorVelocityRotsPerSec = _rollerMotorVelocity.getValue();
        inputs.rollerMotorTorqueCurrentAmps = _rollerMotorTorqueCurrent.getValue();
        inputs.rollerMotorTempCelsius = _rollerMotorDeviceTemp.getValue();

        inputs.deployMotorPositionRots = _deployMotorPosition.getValue();
        inputs.deployMotorVelocityRotsPerSec = _deployMotorVelocity.getValue();
        inputs.deployMotorTorqueCurrentAmps = _deployMotorTorqueCurrent.getValue();
        inputs.deployMotorTempCelsius = _deployMotorDeviceTemp.getValue();
        inputs.deployMotorLimitSwitch = _deployMotorLimitSwitch.getValue().toString();
    }

    @Override
    public void config() {
        final TalonFXConfiguration rollerMotorConfig = new TalonFXConfiguration();
        rollerMotorConfig.Slot0 = new Slot0Configs().withKP(3);
        rollerMotorConfig.CurrentLimits.StatorCurrentLimit = 30;
        rollerMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        rollerMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        rollerMotorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        //TODO set ratio
        rollerMotorConfig.Feedback.SensorToMechanismRatio = 1.3;
        rollerMotor.getConfigurator().apply(rollerMotorConfig);

        final TalonFXConfiguration deployMotorConfig = new TalonFXConfiguration();
        deployMotorConfig.Slot0 = new Slot0Configs()
                .withKP(3)
                .withKG(1);
        deployMotorConfig.TorqueCurrent.PeakForwardTorqueCurrent = 20;
        deployMotorConfig.TorqueCurrent.PeakReverseTorqueCurrent = -20;
        deployMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        deployMotorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        //TODO set ratio
        deployMotorConfig.Feedback.SensorToMechanismRatio = 1.3;
        deployMotorConfig.HardwareLimitSwitch.ReverseLimitSource = ReverseLimitSourceValue.LimitSwitchPin;
        deployMotorConfig.HardwareLimitSwitch.ReverseLimitType = ReverseLimitTypeValue.NormallyOpen;
        deployMotorConfig.HardwareLimitSwitch.ReverseLimitAutosetPositionValue = 0;
        deployMotorConfig.HardwareLimitSwitch.ReverseLimitEnable = true;
        //TODO tune
        deployMotorConfig.MotionMagic.MotionMagicCruiseVelocity = 30;
        deployMotorConfig.MotionMagic.MotionMagicExpo_kV = 1;
        deployMotorConfig.MotionMagic.MotionMagicExpo_kA = 1;
        deployMotor.getConfigurator().apply(deployMotorConfig);
    }


}
