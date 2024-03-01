package frc.robot.subsystems.intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.constants.HardwareConstants;

public class IntakeIOReal implements IntakeIO {
    private final HardwareConstants.IntakeConstants intakeConstants;

    private final TalonFX intakeFrontRollers;
    private final TalonFX intakeBackRollers;
    private final TalonFX shooterFeederRoller;

    private final VelocityTorqueCurrentFOC velocityTorqueCurrentFOC;
    private final TorqueCurrentFOC torqueCurrentFOC;

    private final StatusSignal<Double> _intakeFrontPosition;
    private final StatusSignal<Double> _intakeFrontVelocity;
    private final StatusSignal<Double> _intakeFrontVoltage;
    private final StatusSignal<Double> _intakeFrontTorqueCurrent;
    private final StatusSignal<Double> _intakeFrontDeviceTemp;

    private final StatusSignal<Double> _intakeBackPosition;
    private final StatusSignal<Double> _intakeBackVelocity;
    private final StatusSignal<Double> _intakeBackVoltage;
    private final StatusSignal<Double> _intakeBackTorqueCurrent;
    private final StatusSignal<Double> _intakeBackDeviceTemp;

    private final StatusSignal<Double> _shooterFeederPosition;
    private final StatusSignal<Double> _shooterFeederVelocity;
    private final StatusSignal<Double> _shooterFeederVoltage;
    private final StatusSignal<Double> _shooterFeederTorqueCurrent;
    private final StatusSignal<Double> _shooterFeederDeviceTemp;

    public IntakeIOReal(final HardwareConstants.IntakeConstants intakeConstants) {
        this.intakeConstants = intakeConstants;

        this.intakeFrontRollers = new TalonFX(intakeConstants.intakeFrontMotor(), intakeConstants.CANBus());
        this.intakeBackRollers = new TalonFX(intakeConstants.intakeBackMotor(), intakeConstants.CANBus());
        this.shooterFeederRoller = new TalonFX(intakeConstants.shooterFeederMotor(), intakeConstants.CANBus());

        this.velocityTorqueCurrentFOC = new VelocityTorqueCurrentFOC(0);
        this.torqueCurrentFOC = new TorqueCurrentFOC(0);

        this._intakeFrontPosition = intakeFrontRollers.getPosition();
        this._intakeFrontVelocity = intakeFrontRollers.getVelocity();
        this._intakeFrontVoltage = intakeFrontRollers.getMotorVoltage();
        this._intakeFrontTorqueCurrent = intakeFrontRollers.getTorqueCurrent();
        this._intakeFrontDeviceTemp = intakeFrontRollers.getDeviceTemp();

        this._intakeBackPosition = intakeBackRollers.getPosition();
        this._intakeBackVelocity = intakeBackRollers.getVelocity();
        this._intakeBackVoltage = intakeBackRollers.getMotorVoltage();
        this._intakeBackTorqueCurrent = intakeBackRollers.getTorqueCurrent();
        this._intakeBackDeviceTemp = intakeBackRollers.getDeviceTemp();

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
                _intakeFrontPosition,
                _intakeFrontVelocity,
                _intakeFrontVoltage,
                _intakeFrontTorqueCurrent,
                _intakeFrontDeviceTemp,
                _intakeBackPosition,
                _intakeBackVelocity,
                _intakeBackVoltage,
                _intakeBackTorqueCurrent,
                _intakeBackDeviceTemp,
                _shooterFeederPosition,
                _shooterFeederVelocity,
                _shooterFeederVoltage,
                _shooterFeederTorqueCurrent,
                _shooterFeederDeviceTemp
        );

        inputs.intakeFrontMotorPositionRots = _intakeFrontPosition.getValue();
        inputs.intakeFrontMotorVelocityRotsPerSec = _intakeFrontVelocity.getValue();
        inputs.intakeFrontMotorVoltage = _intakeFrontVoltage.getValue();
        inputs.intakeFrontMotorTorqueCurrentAmps = _intakeFrontTorqueCurrent.getValue();
        inputs.intakeFrontMotorTempCelsius = _intakeFrontDeviceTemp.getValue();

        inputs.intakeBackMotorPositionRots = _intakeBackPosition.getValue();
        inputs.intakeBackVelocityRotsPerSec = _intakeBackVelocity.getValue();
        inputs.intakeBackMotorVoltage = _intakeBackVoltage.getValue();
        inputs.intakeBackTorqueCurrentAmps = _intakeBackTorqueCurrent.getValue();
        inputs.intakeBackTempCelsius = _intakeBackDeviceTemp.getValue();

        inputs.shooterFeederMotorPositionRots = _shooterFeederPosition.getValue();
        inputs.shooterFeederMotorVelocityRotsPerSec = _shooterFeederVelocity.getValue();
        inputs.shooterFeederMotorVoltage = _shooterFeederVoltage.getValue();
        inputs.shooterFeederMotorTorqueCurrentAmps = _shooterFeederTorqueCurrent.getValue();
        inputs.shooterFeederMotorTempCelsius = _shooterFeederDeviceTemp.getValue();
    }

    @SuppressWarnings("DuplicatedCode")
    @Override
    public void config() {
        final TalonFXConfiguration intakeFrontConfig = new TalonFXConfiguration();
        intakeFrontConfig.Slot0 = new Slot0Configs()
                .withKP(0)
                .withKA(0)
                .withKV(0)
                .withKS(0);
        intakeFrontConfig.TorqueCurrent.PeakForwardTorqueCurrent = 80;
        intakeFrontConfig.TorqueCurrent.PeakReverseTorqueCurrent = -80;
        intakeFrontConfig.CurrentLimits.StatorCurrentLimit = 60;
        intakeFrontConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        intakeFrontConfig.CurrentLimits.SupplyCurrentLimit = 50;
        intakeFrontConfig.CurrentLimits.SupplyCurrentThreshold = 2;
        intakeFrontConfig.CurrentLimits.SupplyTimeThreshold = 0.5;
        intakeFrontConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        intakeFrontConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        intakeFrontConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        intakeFrontConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        intakeFrontConfig.Feedback.SensorToMechanismRatio = intakeConstants.intakeFrontRollersGearing();
        intakeFrontRollers.getConfigurator().apply(intakeFrontConfig);

        final TalonFXConfiguration intakeBackConfig = new TalonFXConfiguration();
        intakeBackConfig.Slot0 = new Slot0Configs()
                .withKP(0)
                .withKA(0)
                .withKV(0)
                .withKS(0);
        intakeBackConfig.TorqueCurrent.PeakForwardTorqueCurrent = 80;
        intakeBackConfig.TorqueCurrent.PeakReverseTorqueCurrent = -80;
        intakeBackConfig.CurrentLimits.StatorCurrentLimit = 60;
        intakeBackConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        intakeBackConfig.CurrentLimits.SupplyCurrentLimit = 50;
        intakeBackConfig.CurrentLimits.SupplyCurrentThreshold = 2;
        intakeBackConfig.CurrentLimits.SupplyTimeThreshold = 0.5;
        intakeBackConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        intakeBackConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        intakeBackConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        intakeBackConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        intakeBackConfig.Feedback.SensorToMechanismRatio = intakeConstants.intakeBackRollersGearing();
        intakeBackRollers.getConfigurator().apply(intakeBackConfig);

        final TalonFXConfiguration shooterFollowerConfig = new TalonFXConfiguration();
        shooterFollowerConfig.Slot0 = new Slot0Configs()
                .withKP(0)
                .withKA(0)
                .withKV(0)
                .withKS(0);
        shooterFollowerConfig.TorqueCurrent.PeakForwardTorqueCurrent = 80;
        shooterFollowerConfig.TorqueCurrent.PeakReverseTorqueCurrent = -80;
        shooterFollowerConfig.CurrentLimits.StatorCurrentLimit = 60;
        shooterFollowerConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        shooterFollowerConfig.CurrentLimits.SupplyCurrentLimit = 50;
        shooterFollowerConfig.CurrentLimits.SupplyCurrentThreshold = 2;
        shooterFollowerConfig.CurrentLimits.SupplyTimeThreshold = 0.5;
        shooterFollowerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        shooterFollowerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        shooterFollowerConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        shooterFollowerConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        shooterFollowerConfig.Feedback.SensorToMechanismRatio = intakeConstants.shooterFeederRollerGearing();
        shooterFeederRoller.getConfigurator().apply(shooterFollowerConfig);

        BaseStatusSignal.setUpdateFrequencyForAll(
                100,
                _intakeFrontPosition,
                _intakeFrontVelocity,
                _intakeFrontVoltage,
                _intakeFrontTorqueCurrent,
                _intakeBackPosition,
                _intakeBackVelocity,
                _intakeBackVoltage,
                _intakeBackTorqueCurrent,
                _shooterFeederPosition,
                _shooterFeederVelocity,
                _shooterFeederVoltage,
                _shooterFeederTorqueCurrent

        );
        BaseStatusSignal.setUpdateFrequencyForAll(
                4,
                _intakeFrontDeviceTemp,
                _intakeBackDeviceTemp,
                _shooterFeederDeviceTemp
        );
        ParentDevice.optimizeBusUtilizationForAll(
                intakeFrontRollers,
                intakeBackRollers,
                shooterFeederRoller
        );
    }

    @Override
    public void toVelocity(
            final double frontRollersVelocity,
            final double backRollersVelocity,
            final double shooterFeederRollerVelocity
    ) {
        intakeFrontRollers.setControl(velocityTorqueCurrentFOC.withVelocity(frontRollersVelocity));
        intakeBackRollers.setControl(velocityTorqueCurrentFOC.withVelocity(backRollersVelocity));
        shooterFeederRoller.setControl(velocityTorqueCurrentFOC.withVelocity(shooterFeederRollerVelocity));
    }

    @Override
    public void toTorqueCurrent(
            final double frontRollersTorqueCurrentAmp,
            final double backRollersTorqueCurrentAmp,
            final double shooterFeederRollerTorqueCurrentAmp
    ) {
        intakeFrontRollers.setControl(torqueCurrentFOC.withOutput(frontRollersTorqueCurrentAmp));
        intakeBackRollers.setControl(torqueCurrentFOC.withOutput(backRollersTorqueCurrentAmp));
        shooterFeederRoller.setControl(torqueCurrentFOC.withOutput(shooterFeederRollerTorqueCurrentAmp));
    }
}
