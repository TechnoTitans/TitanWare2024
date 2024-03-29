package frc.robot.subsystems.superstructure.shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
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

public class ShooterIOSim implements ShooterIO {
    private static final double SIM_UPDATE_PERIOD_SEC = 0.005;

    private final DeltaTime deltaTime;
    private final HardwareConstants.ShooterConstants shooterConstants;

    private final TalonFX ampMotor;
    private final TalonFX leftFlywheelMotor;
    private final TalonFX rightFlywheelMotor;

    private final TalonFXSim ampMotorSim;
    private final TalonFXSim leftMotorSim;
    private final TalonFXSim rightMotorSim;

    private final VelocityTorqueCurrentFOC velocityTorqueCurrentFOC;
    private final TorqueCurrentFOC torqueCurrentFOC;
    private final VoltageOut voltageOut;

    // Cached StatusSignals
    private final StatusSignal<Double> _ampPosition;
    private final StatusSignal<Double> _ampVelocity;
    private final StatusSignal<Double> _ampVoltage;
    private final StatusSignal<Double> _ampTorqueCurrent;
    private final StatusSignal<Double> _ampDeviceTemp;
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

    public ShooterIOSim(final HardwareConstants.ShooterConstants shooterConstants) {
        this.deltaTime = new DeltaTime(true);
        this.shooterConstants = shooterConstants;

        this.ampMotor = new TalonFX(shooterConstants.ampMotorId(), shooterConstants.CANBus());
        this.leftFlywheelMotor = new TalonFX(shooterConstants.leftFlywheelMotorId(), shooterConstants.CANBus());
        this.rightFlywheelMotor = new TalonFX(shooterConstants.rightFlywheelMotorId(), shooterConstants.CANBus());

        final DCMotorSim ampFlywheelSim = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(
                        0.121 / (2 * Math.PI),
                        0.01 / (2 * Math.PI)
                ),
                DCMotor.getFalcon500Foc(1),
                shooterConstants.leftFlywheelGearing()
        );

        this.ampMotorSim = new TalonFXSim(
                ampMotor,
                shooterConstants.ampMotorGearing(),
                ampFlywheelSim::update,
                ampFlywheelSim::setInputVoltage,
                ampFlywheelSim::getAngularPositionRad,
                ampFlywheelSim::getAngularVelocityRadPerSec
        );

        final DCMotorSim leftFlywheelSim = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(
                        0.121 / (2 * Math.PI),
                        0.01 / (2 * Math.PI)
                ),
                DCMotor.getFalcon500Foc(1),
                shooterConstants.leftFlywheelGearing()
        );

        this.leftMotorSim = new TalonFXSim(
                leftFlywheelMotor,
                shooterConstants.leftFlywheelGearing(),
                leftFlywheelSim::update,
                leftFlywheelSim::setInputVoltage,
                leftFlywheelSim::getAngularPositionRad,
                leftFlywheelSim::getAngularVelocityRadPerSec
        );

        final DCMotorSim rightFlywheelSim = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(
                        0.122 / (2 * Math.PI),
                        0.011 / (2 * Math.PI)
                ),
                DCMotor.getFalcon500Foc(1),
                shooterConstants.rightFlywheelGearing()
        );

        this.rightMotorSim = new TalonFXSim(
                rightFlywheelMotor,
                shooterConstants.rightFlywheelGearing(),
                rightFlywheelSim::update,
                rightFlywheelSim::setInputVoltage,
                rightFlywheelSim::getAngularPositionRad,
                rightFlywheelSim::getAngularVelocityRadPerSec
        );

        this.velocityTorqueCurrentFOC = new VelocityTorqueCurrentFOC(0);
        this.torqueCurrentFOC = new TorqueCurrentFOC(0);
        this.voltageOut = new VoltageOut(0);

        this._ampPosition = ampMotor.getPosition();
        this._ampVelocity = ampMotor.getVelocity();
        this._ampVoltage = ampMotor.getMotorVoltage();
        this._ampTorqueCurrent = ampMotor.getTorqueCurrent();
        this._ampDeviceTemp = ampMotor.getDeviceTemp();
        this._leftPosition = leftFlywheelMotor.getPosition();
        this._leftVelocity = leftFlywheelMotor.getVelocity();
        this._leftVoltage = leftFlywheelMotor.getMotorVoltage();
        this._leftTorqueCurrent = leftFlywheelMotor.getTorqueCurrent();
        this._leftDeviceTemp = leftFlywheelMotor.getDeviceTemp();
        this._rightPosition = rightFlywheelMotor.getPosition();
        this._rightVelocity = rightFlywheelMotor.getVelocity();
        this._rightVoltage = rightFlywheelMotor.getMotorVoltage();
        this._rightTorqueCurrent = rightFlywheelMotor.getTorqueCurrent();
        this._rightDeviceTemp = rightFlywheelMotor.getDeviceTemp();

        final Notifier simUpdateNotifier = new Notifier(() -> {
            final double dt = deltaTime.get();
            ampMotorSim.update(dt);
            leftMotorSim.update(dt);
            rightMotorSim.update(dt);
        });
        ToClose.add(simUpdateNotifier);
        simUpdateNotifier.setName(String.format(
                "SimUpdate(%d,%d,%d)",
                ampMotor.getDeviceID(),
                leftFlywheelMotor.getDeviceID(),
                rightFlywheelMotor.getDeviceID()
        ));
        simUpdateNotifier.startPeriodic(SIM_UPDATE_PERIOD_SEC);
    }

    @SuppressWarnings("DuplicatedCode")
    @Override
    public void config() {
        final TalonFXConfiguration ampTalonFXConfiguration = new TalonFXConfiguration();
        final InvertedValue ampTalonFXInverted = InvertedValue.Clockwise_Positive;
        ampTalonFXConfiguration.Slot0 = new Slot0Configs()
                .withKS(0)
                .withKV(1.5262)
                .withKA(0.24772)
                .withKP(15.568);
        ampTalonFXConfiguration.TorqueCurrent.PeakForwardTorqueCurrent = 120;
        ampTalonFXConfiguration.TorqueCurrent.PeakReverseTorqueCurrent = -120;
        ampTalonFXConfiguration.CurrentLimits.StatorCurrentLimit = 60;
        ampTalonFXConfiguration.CurrentLimits.StatorCurrentLimitEnable = true;
        ampTalonFXConfiguration.CurrentLimits.SupplyCurrentLimit = 40;
        ampTalonFXConfiguration.CurrentLimits.SupplyCurrentThreshold = 50;
        ampTalonFXConfiguration.CurrentLimits.SupplyTimeThreshold = 0.5;
        ampTalonFXConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;
        ampTalonFXConfiguration.Feedback.SensorToMechanismRatio = shooterConstants.ampMotorGearing();
        ampTalonFXConfiguration.MotorOutput.Inverted = ampTalonFXInverted;
        ampTalonFXConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        ampMotor.getConfigurator().apply(ampTalonFXConfiguration);

        final TalonFXConfiguration leftTalonFXConfiguration = new TalonFXConfiguration();
        final InvertedValue leftTalonFXInverted = InvertedValue.Clockwise_Positive;
        leftTalonFXConfiguration.Slot0 = new Slot0Configs()
                .withKS(0)
                .withKV(1.5238)
                .withKA(0.25265)
                .withKP(15.309);
        leftTalonFXConfiguration.TorqueCurrent.PeakForwardTorqueCurrent = 120;
        leftTalonFXConfiguration.TorqueCurrent.PeakReverseTorqueCurrent = -120;
        leftTalonFXConfiguration.CurrentLimits.StatorCurrentLimit = 60;
        leftTalonFXConfiguration.CurrentLimits.StatorCurrentLimitEnable = true;
        leftTalonFXConfiguration.CurrentLimits.SupplyCurrentLimit = 40;
        leftTalonFXConfiguration.CurrentLimits.SupplyCurrentThreshold = 50;
        leftTalonFXConfiguration.CurrentLimits.SupplyTimeThreshold = 0.5;
        leftTalonFXConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;
        leftTalonFXConfiguration.Feedback.SensorToMechanismRatio = shooterConstants.leftFlywheelGearing();
        leftTalonFXConfiguration.MotorOutput.Inverted = leftTalonFXInverted;
        leftTalonFXConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        leftFlywheelMotor.getConfigurator().apply(leftTalonFXConfiguration);

        final TalonFXConfiguration rightTalonFXConfiguration = new TalonFXConfiguration();
        final InvertedValue rightTalonFXInverted = InvertedValue.CounterClockwise_Positive;
        rightTalonFXConfiguration.Slot0 = new Slot0Configs()
                .withKS(0)
                .withKV(1.5573)
                .withKA(0.26688)
                .withKP(15.575);
        rightTalonFXConfiguration.TorqueCurrent.PeakForwardTorqueCurrent = 120;
        rightTalonFXConfiguration.TorqueCurrent.PeakReverseTorqueCurrent = -120;
        rightTalonFXConfiguration.CurrentLimits.StatorCurrentLimit = 60;
        rightTalonFXConfiguration.CurrentLimits.StatorCurrentLimitEnable = true;
        rightTalonFXConfiguration.CurrentLimits.SupplyCurrentLimit = 40;
        rightTalonFXConfiguration.CurrentLimits.SupplyCurrentThreshold = 50;
        rightTalonFXConfiguration.CurrentLimits.SupplyTimeThreshold = 0.5;
        rightTalonFXConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;
        rightTalonFXConfiguration.Feedback.SensorToMechanismRatio = shooterConstants.rightFlywheelGearing();
        rightTalonFXConfiguration.MotorOutput.Inverted = rightTalonFXInverted;
        rightTalonFXConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        rightFlywheelMotor.getConfigurator().apply(rightTalonFXConfiguration);

        BaseStatusSignal.setUpdateFrequencyForAll(
                100,
                _ampVelocity,
                _ampVoltage,
                _ampTorqueCurrent,
                _leftVelocity,
                _leftVoltage,
                _leftTorqueCurrent,
                _rightVelocity,
                _rightVoltage,
                _rightTorqueCurrent
        );
        BaseStatusSignal.setUpdateFrequencyForAll(
                4,
                _ampPosition,
                _ampDeviceTemp,
                _leftPosition,
                _leftDeviceTemp,
                _rightPosition,
                _rightDeviceTemp
        );
        ParentDevice.optimizeBusUtilizationForAll(
                ampMotor,
                leftFlywheelMotor,
                rightFlywheelMotor
        );

        SimUtils.setCTRETalonFXSimStateMotorInverted(ampMotor, ampTalonFXInverted);
        SimUtils.setCTRETalonFXSimStateMotorInverted(leftFlywheelMotor, leftTalonFXInverted);
        SimUtils.setCTRETalonFXSimStateMotorInverted(rightFlywheelMotor, rightTalonFXInverted);
    }

    @SuppressWarnings("DuplicatedCode")
    @Override
    public void updateInputs(final ShooterIOInputs inputs) {
        BaseStatusSignal.refreshAll(
                _ampPosition,
                _ampVelocity,
                _ampVoltage,
                _ampTorqueCurrent,
                _ampDeviceTemp,
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

        inputs.ampPositionRots = _ampPosition.getValue();
        inputs.ampVelocityRotsPerSec = _ampVelocity.getValue();
        inputs.ampVoltage = _ampVoltage.getValue();
        inputs.ampCurrentAmps = _ampTorqueCurrent.getValue();
        inputs.ampTempCelsius = _ampDeviceTemp.getValue();
        inputs.leftPositionRots = _leftPosition.getValue();
        inputs.leftVelocityRotsPerSec = _leftVelocity.getValue();
        inputs.leftVoltage = _leftVoltage.getValue();
        inputs.leftCurrentAmps = _leftTorqueCurrent.getValue();
        inputs.leftTempCelsius = _leftDeviceTemp.getValue();
        inputs.rightPositionRots = _rightPosition.getValue();
        inputs.rightVelocityRotsPerSec = _rightVelocity.getValue();
        inputs.rightVoltageVolts = _rightVoltage.getValue();
        inputs.rightCurrentAmps = _rightTorqueCurrent.getValue();
        inputs.rightTempCelsius = _rightDeviceTemp.getValue();
    }

    @Override
    public void toVelocity(
            final double ampVelocity,
            final double leftFlywheelVelocity,
            final double rightFlywheelVelocity
    ) {
        ampMotor.setControl(velocityTorqueCurrentFOC.withVelocity(ampVelocity));
        leftFlywheelMotor.setControl(velocityTorqueCurrentFOC.withVelocity(leftFlywheelVelocity));
        rightFlywheelMotor.setControl(velocityTorqueCurrentFOC.withVelocity(rightFlywheelVelocity));
    }

    @Override
    public void toVoltage(
            final double ampVolts,
            final double leftVolts,
            final double rightVolts
    ) {
        ampMotor.setControl(voltageOut.withOutput(ampVolts));
        leftFlywheelMotor.setControl(voltageOut.withOutput(leftVolts));
        rightFlywheelMotor.setControl(voltageOut.withOutput(rightVolts));
    }

    @Override
    public void toTorqueCurrent(
            final double ampTorqueCurrentAmps,
            final double leftTorqueCurrentAmps,
            final double rightTorqueCurrentAmps
    ) {
        ampMotor.setControl(torqueCurrentFOC.withOutput(ampTorqueCurrentAmps));
        leftFlywheelMotor.setControl(torqueCurrentFOC.withOutput(leftTorqueCurrentAmps));
        rightFlywheelMotor.setControl(torqueCurrentFOC.withOutput(rightTorqueCurrentAmps));
    }
}
