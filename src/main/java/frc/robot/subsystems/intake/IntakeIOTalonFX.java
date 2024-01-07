package frc.robot.subsystems.intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ForwardLimitSourceValue;
import com.ctre.phoenix6.signals.ReverseLimitSourceValue;
import com.ctre.phoenix6.signals.ReverseLimitTypeValue;
import frc.robot.constants.HardwareConstants;

public class IntakeIOTalonFX implements IntakeIO {
    private final TalonFX rollerMotor;

    private final VelocityVoltage velocityVoltage;

    private final StatusSignal<Double> _rollerMotorPosition;
    private final StatusSignal<Double> _rollerMotorVelocity;
    private final StatusSignal<Double> _rollerMotorTorqueCurrent;
    private final StatusSignal<Double> _rollerMotorDeviceTemp;

    public IntakeIOTalonFX(final HardwareConstants.IntakeConstants intakeConstants) {
        this.rollerMotor = new TalonFX(intakeConstants.intakeRollerMotorId(), intakeConstants.intakeCANBus());

        this.velocityVoltage = new VelocityVoltage(0);

        this._rollerMotorPosition = rollerMotor.getPosition();
        this._rollerMotorVelocity = rollerMotor.getVelocity();
        this._rollerMotorTorqueCurrent = rollerMotor.getTorqueCurrent();
        this._rollerMotorDeviceTemp = rollerMotor.getDeviceTemp();
    }

    @Override
    public void periodic() {
        BaseStatusSignal.refreshAll(
                _rollerMotorPosition,
                _rollerMotorVelocity,
                _rollerMotorTorqueCurrent,
                _rollerMotorDeviceTemp
        );
    }

    @Override
    public void updateInputs(final IntakeIOInputs inputs) {
        inputs.rollerMotorPositionRots = _rollerMotorPosition.getValue();
        inputs.rollerMotorVelocityRotsPerSec = _rollerMotorVelocity.getValue();
        inputs.rollerMotorTorqueCurrentAmps = _rollerMotorTorqueCurrent.getValue();
        inputs.rollerMotorTempCelsius = _rollerMotorDeviceTemp.getValue();
    }

    @Override
    public void config() {
        final TalonFXConfiguration rollerMotorConfig = new TalonFXConfiguration();
        rollerMotorConfig.Slot0 = new Slot0Configs().withKP(3);
        rollerMotorConfig.HardwareLimitSwitch.ReverseLimitSource = ReverseLimitSourceValue.LimitSwitchPin;
        rollerMotorConfig.HardwareLimitSwitch.ReverseLimitType = ReverseLimitTypeValue.NormallyOpen;
        rollerMotorConfig.HardwareLimitSwitch.ReverseLimitEnable = true;
        rollerMotorConfig.HardwareLimitSwitch.ReverseLimitAutosetPositionValue = 0;
        rollerMotorConfig.HardwareLimitSwitch.ReverseLimitAutosetPositionEnable = true;
        rollerMotorConfig.CurrentLimits.StatorCurrentLimit = 30;
        rollerMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        rollerMotor.getConfigurator().apply(rollerMotorConfig);
    }
}
