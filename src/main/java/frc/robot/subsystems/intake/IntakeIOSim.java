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
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.constants.HardwareConstants;
import frc.robot.utils.closeables.ToClose;
import frc.robot.utils.control.DeltaTime;
import frc.robot.utils.sim.SimUtils;
import frc.robot.utils.sim.motors.TalonFXSim;

public class IntakeIOSim implements IntakeIO {
    private static final double SIM_UPDATE_PERIOD_SEC = 0.005;

    private final HardwareConstants.IntakeConstants intakeConstants;
    private final DeltaTime deltaTime;

    private final TalonFX intakeFrontRollers;
    private final TalonFX intakeBackRollers;
    private final TalonFX shooterFeederRoller;

    private final TalonFXSim intakeFrontRollersSim;
    private final TalonFXSim intakeBackRollersSim;
    private final TalonFXSim shooterFeederRollerSim;

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

    public IntakeIOSim(final HardwareConstants.IntakeConstants intakeConstants) {
        this.intakeConstants = intakeConstants;
        this.deltaTime = new DeltaTime(true);

        this.intakeFrontRollers = new TalonFX(intakeConstants.intakeFrontMotor(), intakeConstants.CANBus());
        this.intakeBackRollers = new TalonFX(intakeConstants.intakeBackMotor(), intakeConstants.CANBus());
        this.shooterFeederRoller = new TalonFX(intakeConstants.shooterFeederMotor(), intakeConstants.CANBus());

        final DCMotorSim frontRollerSim = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(
                        0.18121 / (2 * Math.PI),
                        2.9787 / (2 * Math.PI)
                ),
                DCMotor.getKrakenX60Foc(1),
                intakeConstants.intakeBackRollersGearing()
        );

        this.intakeFrontRollersSim = new TalonFXSim(
                intakeFrontRollers,
                intakeConstants.intakeFrontRollersGearing(),
                frontRollerSim::update,
                frontRollerSim::setInputVoltage,
                frontRollerSim::getAngularPositionRad,
                frontRollerSim::getAngularVelocityRadPerSec
        );

        final DCMotorSim backRollerSim = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(
                        0.19557 / (2 * Math.PI),
                        2.9856 / (2 * Math.PI)
                ),
                DCMotor.getKrakenX60Foc(1),
                intakeConstants.intakeBackRollersGearing()
        );

        this.intakeBackRollersSim = new TalonFXSim(
                intakeBackRollers,
                intakeConstants.intakeBackRollersGearing(),
                backRollerSim::update,
                backRollerSim::setInputVoltage,
                backRollerSim::getAngularPositionRad,
                backRollerSim::getAngularVelocityRadPerSec
        );

        final DCMotorSim shooterFeederSim = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(
                        3.1779 / (2 * Math.PI),
                        5.1848 / (2 * Math.PI)
                ),
                DCMotor.getKrakenX60Foc(1),
                intakeConstants.intakeBackRollersGearing()
        );

        this.shooterFeederRollerSim = new TalonFXSim(
                shooterFeederRoller,
                intakeConstants.shooterFeederRollerGearing(),
                shooterFeederSim::update,
                shooterFeederSim::setInputVoltage,
                shooterFeederSim::getAngularPositionRad,
                shooterFeederSim::getAngularVelocityRadPerSec
        );

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

        final Notifier simUpdateNotifier = new Notifier(() -> {
            final double dt = deltaTime.get();
            intakeFrontRollersSim.update(dt);
            intakeBackRollersSim.update(dt);
            shooterFeederRollerSim.update(dt);
        });
        ToClose.add(simUpdateNotifier);
        simUpdateNotifier.setName(String.format(
                "SimUpdate(%d,%d,%d)",
                intakeFrontRollers.getDeviceID(),
                intakeBackRollers.getDeviceID(),
                shooterFeederRoller.getDeviceID()
        ));
        simUpdateNotifier.startPeriodic(SIM_UPDATE_PERIOD_SEC);
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
        final InvertedValue intakeFrontInvertedValue = InvertedValue.CounterClockwise_Positive;
        final TalonFXConfiguration intakeFrontConfig = new TalonFXConfiguration();
        intakeFrontConfig.Slot0 = new Slot0Configs()
                .withKS(0)
                .withKV(0.18121)
                .withKA(2.9787)
                .withKP(3.6111);
        intakeFrontConfig.TorqueCurrent.PeakForwardTorqueCurrent = 80;
        intakeFrontConfig.TorqueCurrent.PeakReverseTorqueCurrent = -80;
        intakeFrontConfig.CurrentLimits.StatorCurrentLimit = 60;
        intakeFrontConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        intakeFrontConfig.CurrentLimits.SupplyCurrentLimit = 40;
        intakeFrontConfig.CurrentLimits.SupplyCurrentThreshold = 55;
        intakeFrontConfig.CurrentLimits.SupplyTimeThreshold = 1.5;
        intakeFrontConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        intakeFrontConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        intakeFrontConfig.MotorOutput.Inverted = intakeFrontInvertedValue;
        intakeFrontConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        intakeFrontConfig.Feedback.SensorToMechanismRatio = intakeConstants.intakeFrontRollersGearing();
        intakeFrontRollers.getConfigurator().apply(intakeFrontConfig);

        final InvertedValue intakeBackInvertedValue = InvertedValue.Clockwise_Positive;
        final TalonFXConfiguration intakeBackConfig = new TalonFXConfiguration();
        intakeBackConfig.Slot0 = new Slot0Configs()
                .withKS(0)
                .withKV(0.19557)
                .withKA(2.9856)
                .withKP(1.955);
        intakeBackConfig.TorqueCurrent.PeakForwardTorqueCurrent = 80;
        intakeBackConfig.TorqueCurrent.PeakReverseTorqueCurrent = -80;
        intakeBackConfig.CurrentLimits.StatorCurrentLimit = 60;
        intakeBackConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        intakeBackConfig.CurrentLimits.SupplyCurrentLimit = 40;
        intakeBackConfig.CurrentLimits.SupplyCurrentThreshold = 55;
        intakeBackConfig.CurrentLimits.SupplyTimeThreshold = 1.5;
        intakeBackConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        intakeBackConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        intakeBackConfig.MotorOutput.Inverted = intakeBackInvertedValue;
        intakeBackConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        intakeBackConfig.Feedback.SensorToMechanismRatio = intakeConstants.intakeBackRollersGearing();
        intakeBackRollers.getConfigurator().apply(intakeBackConfig);

        final InvertedValue shooterFeederInvertedValue = InvertedValue.CounterClockwise_Positive;
        final TalonFXConfiguration shooterFeederConfig = new TalonFXConfiguration();
        shooterFeederConfig.Slot0 = new Slot0Configs()
                .withKS(0)
                .withKV(3.1779)
                .withKA(5.1848)
                .withKP(0.5074);
        shooterFeederConfig.TorqueCurrent.PeakForwardTorqueCurrent = 80;
        shooterFeederConfig.TorqueCurrent.PeakReverseTorqueCurrent = -80;
        shooterFeederConfig.CurrentLimits.StatorCurrentLimit = 60;
        shooterFeederConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        shooterFeederConfig.CurrentLimits.SupplyCurrentLimit = 40;
        shooterFeederConfig.CurrentLimits.SupplyCurrentThreshold = 55;
        shooterFeederConfig.CurrentLimits.SupplyTimeThreshold = 1.5;
        shooterFeederConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        shooterFeederConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        shooterFeederConfig.MotorOutput.Inverted = shooterFeederInvertedValue;
        shooterFeederConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        shooterFeederConfig.Feedback.SensorToMechanismRatio = intakeConstants.shooterFeederRollerGearing();
        shooterFeederRoller.getConfigurator().apply(shooterFeederConfig);

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

        SimUtils.setCTRETalonFXSimStateMotorInverted(intakeFrontRollers, intakeFrontInvertedValue);
        SimUtils.setCTRETalonFXSimStateMotorInverted(intakeBackRollers, intakeBackInvertedValue);
        SimUtils.setCTRETalonFXSimStateMotorInverted(shooterFeederRoller, shooterFeederInvertedValue);
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
