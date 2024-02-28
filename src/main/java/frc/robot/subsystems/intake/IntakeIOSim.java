package frc.robot.subsystems.intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.*;
import frc.robot.constants.HardwareConstants;

public class IntakeIOSim implements IntakeIO {
    private final TalonFX intakeFrontRollers;
    private final TalonFX intakeBackRollers;
    private final TalonFX shooterFeederRoller;

    private final VelocityTorqueCurrentFOC intakeFrontVelocityTorqueFOC;
    private final VelocityTorqueCurrentFOC intakeBackVelocityTorqueFOC;
    private final VelocityTorqueCurrentFOC shooterFeederVelocityTorqueFOC;

    private final StatusSignal<Double> _intakeFrontPosition;
    private final StatusSignal<Double> _intakeFrontVelocity;
    private final StatusSignal<Double> _intakeFrontTorqueCurrent;
    private final StatusSignal<Double> _intakeFrontDeviceTemp;

    private final StatusSignal<Double> _intakeBackPosition;
    private final StatusSignal<Double> _intakeBackVelocity;
    private final StatusSignal<Double> _intakeBackTorqueCurrent;
    private final StatusSignal<Double> _intakeBackDeviceTemp;

    private final StatusSignal<Double> _shooterFeederPosition;
    private final StatusSignal<Double> _shooterFeederVelocity;
    private final StatusSignal<Double> _shooterFeederTorqueCurrent;
    private final StatusSignal<Double> _shooterFeederDeviceTemp;

    public IntakeIOSim(final HardwareConstants.IntakeConstants intakeConstants) {
        this.intakeFrontRollers = new TalonFX(intakeConstants.intakeMotorMaster(), intakeConstants.CANBus());
        this.intakeBackRollers = new TalonFX(intakeConstants.intakeMotorMaster(), intakeConstants.CANBus());
        this.shooterFeederRoller = new TalonFX(intakeConstants.shooterFeederMotor(), intakeConstants.CANBus());

        this.intakeFrontVelocityTorqueFOC = new VelocityTorqueCurrentFOC(0);
        this.intakeBackVelocityTorqueFOC = new VelocityTorqueCurrentFOC(0);
        this.shooterFeederVelocityTorqueFOC = new VelocityTorqueCurrentFOC(0);

        this._intakeFrontPosition = intakeFrontRollers.getPosition();
        this._intakeFrontVelocity = intakeFrontRollers.getVelocity();
        this._intakeFrontTorqueCurrent = intakeFrontRollers.getTorqueCurrent();
        this._intakeFrontDeviceTemp = intakeFrontRollers.getDeviceTemp();

        this._intakeBackPosition = intakeBackRollers.getPosition();
        this._intakeBackVelocity = intakeBackRollers.getVelocity();
        this._intakeBackTorqueCurrent = intakeBackRollers.getTorqueCurrent();
        this._intakeBackDeviceTemp = intakeBackRollers.getDeviceTemp();

        this._shooterFeederPosition = shooterFeederRoller.getPosition();
        this._shooterFeederVelocity = shooterFeederRoller.getVelocity();
        this._shooterFeederTorqueCurrent = shooterFeederRoller.getTorqueCurrent();
        this._shooterFeederDeviceTemp = shooterFeederRoller.getDeviceTemp();
    }

    @Override
    public void updateInputs(final IntakeIOInputs inputs) {
        BaseStatusSignal.refreshAll(
                _intakeFrontPosition,
                _intakeFrontVelocity,
                _intakeFrontTorqueCurrent,
                _intakeFrontDeviceTemp,
                _intakeBackPosition,
                _intakeBackVelocity,
                _intakeBackTorqueCurrent,
                _intakeBackDeviceTemp,
                _shooterFeederPosition,
                _shooterFeederVelocity,
                _shooterFeederTorqueCurrent,
                _shooterFeederDeviceTemp
        );

        inputs.intakeFrontMotorPositionRots = _intakeFrontPosition.getValue();
        inputs.intakeFrontMotorVelocityRotsPerSec = _intakeFrontVelocity.getValue();
        inputs.intakeFrontMotorTorqueCurrentAmps = _intakeFrontTorqueCurrent.getValue();
        inputs.intakeFrontMotorTempCelsius = _intakeFrontDeviceTemp.getValue();

        inputs.intakeBackMotorPositionRots = _intakeBackPosition.getValue();
        inputs.intakeBackVelocityRotsPerSec = _intakeBackVelocity.getValue();
        inputs.intakeBackTorqueCurrentAmps = _intakeBackTorqueCurrent.getValue();
        inputs.intakeBackTempCelsius = _intakeBackDeviceTemp.getValue();

        inputs.shooterFeederMotorPositionRots = _shooterFeederPosition.getValue();
        inputs.shooterFeederMotorVelocityRotsPerSec = _shooterFeederVelocity.getValue();
        inputs.shooterFeederMotorTorqueCurrentAmps = _shooterFeederTorqueCurrent.getValue();
        inputs.shooterFeederMotorTempCelsius = _shooterFeederDeviceTemp.getValue();
    }

    @Override
    public void config() {
        final TalonFXConfiguration intakeFrontConfig = new TalonFXConfiguration();
        intakeFrontConfig.Slot0 = new Slot0Configs()
                .withKP(3)
                .withKA(1)
                .withKV(1)
                .withKS(1);
        intakeFrontConfig.TorqueCurrent.PeakForwardTorqueCurrent = 40;
        intakeFrontConfig.TorqueCurrent.PeakReverseTorqueCurrent = -40;
        intakeFrontConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        intakeFrontConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        intakeFrontConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        intakeFrontConfig.Feedback.SensorToMechanismRatio = 2;
        intakeFrontRollers.getConfigurator().apply(intakeFrontConfig);

        final TalonFXConfiguration intakeBackConfig = new TalonFXConfiguration();
        intakeBackConfig.Slot0 = new Slot0Configs()
                .withKP(3)
                .withKA(1)
                .withKV(1)
                .withKS(1);
        intakeBackConfig.TorqueCurrent.PeakForwardTorqueCurrent = 40;
        intakeBackConfig.TorqueCurrent.PeakReverseTorqueCurrent = -40;
        intakeBackConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        intakeBackConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        intakeBackConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        intakeBackConfig.Feedback.SensorToMechanismRatio = 2;
        intakeBackRollers.getConfigurator().apply(intakeBackConfig);

        final TalonFXConfiguration shooterFollowerConfig = new TalonFXConfiguration();
        shooterFollowerConfig.Slot0 = new Slot0Configs()
                .withKP(3)
                .withKA(1)
                .withKV(1)
                .withKS(1);
        shooterFollowerConfig.TorqueCurrent.PeakForwardTorqueCurrent = 40;
        shooterFollowerConfig.TorqueCurrent.PeakReverseTorqueCurrent = -40;
        shooterFollowerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        shooterFollowerConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        shooterFollowerConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        shooterFollowerConfig.Feedback.SensorToMechanismRatio = 1;
        shooterFeederRoller.getConfigurator().apply(shooterFollowerConfig);
    }
}
