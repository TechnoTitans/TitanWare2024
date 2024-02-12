package frc.robot.subsystems.intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.*;
import frc.robot.constants.HardwareConstants;

public class IntakeIOTalonFX implements IntakeIO {
    private final TalonFX rollerMotor;

    private final VelocityTorqueCurrentFOC rollerVelocityTorqueFOC;

    private final StatusSignal<Double> _rollerMotorPosition;
    private final StatusSignal<Double> _rollerMotorVelocity;
    private final StatusSignal<Double> _rollerMotorTorqueCurrent;
    private final StatusSignal<Double> _rollerMotorDeviceTemp;

    public IntakeIOTalonFX(final HardwareConstants.IntakeConstants intakeConstants) {
        this.rollerMotor = new TalonFX(intakeConstants.rollerMotorId(), intakeConstants.CANBus());

        this.rollerVelocityTorqueFOC = new VelocityTorqueCurrentFOC(0);

        this._rollerMotorPosition = rollerMotor.getPosition();
        this._rollerMotorVelocity = rollerMotor.getVelocity();
        this._rollerMotorTorqueCurrent = rollerMotor.getTorqueCurrent();
        this._rollerMotorDeviceTemp = rollerMotor.getDeviceTemp();
    }

    @Override
    public void updateInputs(final IntakeIOInputs inputs) {
        BaseStatusSignal.refreshAll(
                _rollerMotorPosition,
                _rollerMotorVelocity,
                _rollerMotorTorqueCurrent,
                _rollerMotorDeviceTemp
        );

        inputs.rollerMotorPositionRots = _rollerMotorPosition.getValue();
        inputs.rollerMotorVelocityRotsPerSec = _rollerMotorVelocity.getValue();
        inputs.rollerMotorTorqueCurrentAmps = _rollerMotorTorqueCurrent.getValue();
        inputs.rollerMotorTempCelsius = _rollerMotorDeviceTemp.getValue();
    }

    @Override
    public void config() {
        final TalonFXConfiguration rollerMotorConfig = new TalonFXConfiguration();
        rollerMotorConfig.Slot0 = new Slot0Configs()
                .withKP(3)
                .withKA(1)
                .withKV(1)
                .withKS(1);
        rollerMotorConfig.TorqueCurrent.PeakForwardTorqueCurrent = 40;
        rollerMotorConfig.TorqueCurrent.PeakReverseTorqueCurrent = -40;
        rollerMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        rollerMotorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        //TODO set ratio
        rollerMotorConfig.Feedback.SensorToMechanismRatio = 1.3;
        rollerMotor.getConfigurator().apply(rollerMotorConfig);
    }
}
