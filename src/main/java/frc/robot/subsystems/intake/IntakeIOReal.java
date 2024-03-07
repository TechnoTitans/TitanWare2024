package frc.robot.subsystems.intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.constants.HardwareConstants;

public class IntakeIOReal implements IntakeIO {
    private final HardwareConstants.IntakeConstants intakeConstants;

    private final TalonFX rightRoller;
    private final TalonFX leftRoller;
    private final TalonFX shooterFeederRoller;

    private final DigitalInput shooterBeamBreak;

    private final VelocityTorqueCurrentFOC velocityTorqueCurrentFOC;
    private final TorqueCurrentFOC torqueCurrentFOC;
    private final VoltageOut voltageOut;

    private final StatusSignal<Double> _rightRollerPosition;
    private final StatusSignal<Double> _rightRollerVelocity;
    private final StatusSignal<Double> _rightRollerVoltage;
    private final StatusSignal<Double> _rightRollerTorqueCurrent;
    private final StatusSignal<Double> _rightRollerDeviceTemp;

    private final StatusSignal<Double> _leftRollerPosition;
    private final StatusSignal<Double> _leftRollerVelocity;
    private final StatusSignal<Double> _leftRollerVoltage;
    private final StatusSignal<Double> _leftRollerTorqueCurrent;
    private final StatusSignal<Double> _leftRollerDeviceTemp;

    private final StatusSignal<Double> _shooterFeederPosition;
    private final StatusSignal<Double> _shooterFeederVelocity;
    private final StatusSignal<Double> _shooterFeederVoltage;
    private final StatusSignal<Double> _shooterFeederTorqueCurrent;
    private final StatusSignal<Double> _shooterFeederDeviceTemp;

    public IntakeIOReal(final HardwareConstants.IntakeConstants intakeConstants) {
        this.intakeConstants = intakeConstants;

        this.rightRoller = new TalonFX(intakeConstants.rightRollerMotor(), intakeConstants.CANBus());
        this.leftRoller = new TalonFX(intakeConstants.leftRollerMotor(), intakeConstants.CANBus());
        this.shooterFeederRoller = new TalonFX(intakeConstants.shooterFeederRollerMotor(), intakeConstants.CANBus());

        this.shooterBeamBreak = new DigitalInput(intakeConstants.sensorDigitalInput());

        this.velocityTorqueCurrentFOC = new VelocityTorqueCurrentFOC(0);
        this.torqueCurrentFOC = new TorqueCurrentFOC(0);
        this.voltageOut = new VoltageOut(0);

        this._rightRollerPosition = rightRoller.getPosition();
        this._rightRollerVelocity = rightRoller.getVelocity();
        this._rightRollerVoltage = rightRoller.getMotorVoltage();
        this._rightRollerTorqueCurrent = rightRoller.getTorqueCurrent();
        this._rightRollerDeviceTemp = rightRoller.getDeviceTemp();

        this._leftRollerPosition = leftRoller.getPosition();
        this._leftRollerVelocity = leftRoller.getVelocity();
        this._leftRollerVoltage = leftRoller.getMotorVoltage();
        this._leftRollerTorqueCurrent = leftRoller.getTorqueCurrent();
        this._leftRollerDeviceTemp = leftRoller.getDeviceTemp();

        this._shooterFeederPosition = shooterFeederRoller.getPosition();
        this._shooterFeederVelocity = shooterFeederRoller.getVelocity();
        this._shooterFeederVoltage = shooterFeederRoller.getMotorVoltage();
        this._shooterFeederTorqueCurrent = shooterFeederRoller.getTorqueCurrent();
        this._shooterFeederDeviceTemp = shooterFeederRoller.getDeviceTemp();
    }

    @SuppressWarnings("DuplicatedCode")
    @Override
    public void updateInputs(final IntakeIOInputs inputs) {
        BaseStatusSignal.refreshAll(
                _rightRollerPosition,
                _rightRollerVelocity,
                _rightRollerVoltage,
                _rightRollerTorqueCurrent,
                _rightRollerDeviceTemp,
                _leftRollerPosition,
                _leftRollerVelocity,
                _leftRollerVoltage,
                _leftRollerTorqueCurrent,
                _leftRollerDeviceTemp,
                _shooterFeederPosition,
                _shooterFeederVelocity,
                _shooterFeederVoltage,
                _shooterFeederTorqueCurrent,
                _shooterFeederDeviceTemp
        );

        inputs.rightPositionRots = _rightRollerPosition.getValue();
        inputs.rightVelocityRotsPerSec = _rightRollerVelocity.getValue();
        inputs.rightVoltage = _rightRollerVoltage.getValue();
        inputs.rightTorqueCurrentAmps = _rightRollerTorqueCurrent.getValue();
        inputs.rightTempCelsius = _rightRollerDeviceTemp.getValue();

        inputs.leftPositionRots = _leftRollerPosition.getValue();
        inputs.leftVelocityRotsPerSec = _leftRollerVelocity.getValue();
        inputs.leftVoltage = _leftRollerVoltage.getValue();
        inputs.leftTorqueCurrentAmps = _leftRollerTorqueCurrent.getValue();
        inputs.leftTempCelsius = _leftRollerDeviceTemp.getValue();

        inputs.shooterFeederPositionRots = _shooterFeederPosition.getValue();
        inputs.shooterFeederVelocityRotsPerSec = _shooterFeederVelocity.getValue();
        inputs.shooterFeederVoltage = _shooterFeederVoltage.getValue();
        inputs.shooterFeederTorqueCurrentAmps= _shooterFeederTorqueCurrent.getValue();
        inputs.shooterFeederTempCelsius = _shooterFeederDeviceTemp.getValue();

        inputs.shooterBeamBreak = !shooterBeamBreak.get();
    }

    @SuppressWarnings("DuplicatedCode")
    @Override
    public void config() {
        final TalonFXConfiguration rightRollerConfig = new TalonFXConfiguration();
        rightRollerConfig.Slot0 = new Slot0Configs()
                .withKP(0)
                .withKA(0)
                .withKV(0)
                .withKS(0);
        rightRollerConfig.TorqueCurrent.PeakForwardTorqueCurrent = 80;
        rightRollerConfig.TorqueCurrent.PeakReverseTorqueCurrent = -80;
        rightRollerConfig.CurrentLimits.StatorCurrentLimit = 60;
        rightRollerConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        rightRollerConfig.CurrentLimits.SupplyCurrentLimit = 40;
        rightRollerConfig.CurrentLimits.SupplyCurrentThreshold = 55;
        rightRollerConfig.CurrentLimits.SupplyTimeThreshold = 1.5;
        rightRollerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        rightRollerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        rightRollerConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        rightRollerConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        rightRollerConfig.Feedback.SensorToMechanismRatio = intakeConstants.rightMotorGearing();
        rightRoller.getConfigurator().apply(rightRollerConfig);

        final TalonFXConfiguration leftRollerConfig = new TalonFXConfiguration();
        leftRollerConfig.Slot0 = new Slot0Configs()
                .withKP(0)
                .withKA(0)
                .withKV(0)
                .withKS(0);
        leftRollerConfig.TorqueCurrent.PeakForwardTorqueCurrent = 80;
        leftRollerConfig.TorqueCurrent.PeakReverseTorqueCurrent = -80;
        leftRollerConfig.CurrentLimits.StatorCurrentLimit = 60;
        leftRollerConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        leftRollerConfig.CurrentLimits.SupplyCurrentLimit = 40;
        leftRollerConfig.CurrentLimits.SupplyCurrentThreshold = 55;
        leftRollerConfig.CurrentLimits.SupplyTimeThreshold = 1.5;
        leftRollerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        leftRollerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        leftRollerConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        leftRollerConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        leftRollerConfig.Feedback.SensorToMechanismRatio = intakeConstants.leftMotorGearing();
        leftRoller.getConfigurator().apply(leftRollerConfig);

        final TalonFXConfiguration shooterFeederConfig = new TalonFXConfiguration();
        shooterFeederConfig.Slot0 = new Slot0Configs()
                .withKP(0)
                .withKA(0)
                .withKV(0)
                .withKS(0);
        shooterFeederConfig.TorqueCurrent.PeakForwardTorqueCurrent = 80;
        shooterFeederConfig.TorqueCurrent.PeakReverseTorqueCurrent = -80;
        shooterFeederConfig.CurrentLimits.StatorCurrentLimit = 80;
        shooterFeederConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        shooterFeederConfig.CurrentLimits.SupplyCurrentLimit = 40;
        shooterFeederConfig.CurrentLimits.SupplyCurrentThreshold = 55;
        shooterFeederConfig.CurrentLimits.SupplyTimeThreshold = 1.5;
        shooterFeederConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        shooterFeederConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        shooterFeederConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        shooterFeederConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        shooterFeederConfig.Feedback.SensorToMechanismRatio = intakeConstants.shooterFeederMotorGearing();
        shooterFeederRoller.getConfigurator().apply(shooterFeederConfig);

        BaseStatusSignal.setUpdateFrequencyForAll(
                100,
                _rightRollerPosition,
                _rightRollerVelocity,
                _rightRollerVoltage,
                _rightRollerTorqueCurrent,
                _leftRollerPosition,
                _leftRollerVelocity,
                _leftRollerVoltage,
                _leftRollerTorqueCurrent,
                _shooterFeederPosition,
                _shooterFeederVelocity,
                _shooterFeederVoltage,
                _shooterFeederTorqueCurrent

        );
        BaseStatusSignal.setUpdateFrequencyForAll(
                4,
                _rightRollerDeviceTemp,
                _leftRollerDeviceTemp,
                _shooterFeederDeviceTemp
        );
        ParentDevice.optimizeBusUtilizationForAll(
                rightRoller,
                leftRoller,
                shooterFeederRoller
        );
    }

    @Override
    public void toVelocity(
            final double rightRollerVelocity,
            final double leftRollerVelocity,
            final double shooterFeederRollerVelocity
    ) {
        rightRoller.setControl(velocityTorqueCurrentFOC.withVelocity(rightRollerVelocity));
        leftRoller.setControl(velocityTorqueCurrentFOC.withVelocity(leftRollerVelocity));
        shooterFeederRoller.setControl(velocityTorqueCurrentFOC.withVelocity(shooterFeederRollerVelocity));
    }

    @Override
    public void toTorqueCurrent(
            final double leftRollerTorqueCurrentAmp,
            final double rightRollerTorqueCurrentAmp,
            final double shooterFeederRollerTorqueCurrentAmp
    ) {
        rightRoller.setControl(torqueCurrentFOC.withOutput(leftRollerTorqueCurrentAmp));
        leftRoller.setControl(torqueCurrentFOC.withOutput(rightRollerTorqueCurrentAmp));
        shooterFeederRoller.setControl(torqueCurrentFOC.withOutput(shooterFeederRollerTorqueCurrentAmp));
    }

    @Override
    public void toVoltage(double rightRollerVolts, double leftRollerVolts, double shooterFeederRollerVolts) {
        rightRoller.setControl(voltageOut.withOutput(rightRollerVolts));
        leftRoller.setControl(voltageOut.withOutput(leftRollerVolts));
        shooterFeederRoller.setControl(voltageOut.withOutput(shooterFeederRollerVolts));
    }
}
