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
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.AsynchronousInterrupt;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.DIOSim;
import frc.robot.constants.HardwareConstants;
import frc.robot.utils.closeables.ToClose;
import frc.robot.utils.control.DeltaTime;
import frc.robot.utils.sim.SimUtils;
import frc.robot.utils.sim.motors.TalonFXSim;

public class IntakeIOSim implements IntakeIO {
    private static final double SIM_UPDATE_PERIOD_SEC = 0.005;

    private final HardwareConstants.IntakeConstants intakeConstants;
    private final DeltaTime deltaTime;

    private final TalonFX rightRoller;
    private final TalonFX leftRoller;
    private final TalonFX shooterFeederRoller;

    private final DigitalInput shooterBeamBreak;
    private final DIOSim shooterBeamBreakSim;
    private final AsynchronousInterrupt shooterBeamBreakInterrupt;

    private final TalonFXSim rightRollerSim;
    private final TalonFXSim leftRollerSim;
    private final TalonFXSim shooterFeederRollerSim;

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

    public IntakeIOSim(final HardwareConstants.IntakeConstants intakeConstants) {
        this.intakeConstants = intakeConstants;
        this.deltaTime = new DeltaTime(true);

        this.rightRoller = new TalonFX(intakeConstants.rightRollerMotor(), intakeConstants.CANBus());
        this.leftRoller = new TalonFX(intakeConstants.leftRollerMotor(), intakeConstants.CANBus());
        this.shooterFeederRoller = new TalonFX(intakeConstants.shooterFeederRollerMotor(), intakeConstants.CANBus());

        this.shooterBeamBreak = new DigitalInput(intakeConstants.sensorDigitalInput());
        this.shooterBeamBreakSim = new DIOSim(shooterBeamBreak);
        this.shooterBeamBreakInterrupt = new AsynchronousInterrupt(
                shooterBeamBreak,
                (final Boolean rising, final Boolean falling) -> {
                    if (falling) {
                        toVoltage(0, 0, 0);
                    }
                });
        
        final DCMotorSim rightRollerMotorSim = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(
                        0.18121 / (2 * Math.PI),
                        2.9787 / (2 * Math.PI)
                ),
                DCMotor.getKrakenX60Foc(1),
                intakeConstants.leftMotorGearing()
        );

        this.rightRollerSim = new TalonFXSim(
                rightRoller,
                intakeConstants.rightMotorGearing(),
                rightRollerMotorSim::update,
                rightRollerMotorSim::setInputVoltage,
                rightRollerMotorSim::getAngularPositionRad,
                rightRollerMotorSim::getAngularVelocityRadPerSec
        );

        final DCMotorSim leftRollerMotorSim = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(
                        0.19557 / (2 * Math.PI),
                        2.9856 / (2 * Math.PI)
                ),
                DCMotor.getKrakenX60Foc(1),
                intakeConstants.leftMotorGearing()
        );

        this.leftRollerSim = new TalonFXSim(
                leftRoller,
                intakeConstants.leftMotorGearing(),
                leftRollerMotorSim::update,
                leftRollerMotorSim::setInputVoltage,
                leftRollerMotorSim::getAngularPositionRad,
                leftRollerMotorSim::getAngularVelocityRadPerSec
        );

        final DCMotorSim shooterFeederMotorSim = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(
                        3.1779 / (2 * Math.PI),
                        5.1848 / (2 * Math.PI)
                ),
                DCMotor.getKrakenX60Foc(1),
                intakeConstants.leftMotorGearing()
        );

        this.shooterFeederRollerSim = new TalonFXSim(
                shooterFeederRoller,
                intakeConstants.shooterFeederMotorGearing(),
                shooterFeederMotorSim::update,
                shooterFeederMotorSim::setInputVoltage,
                shooterFeederMotorSim::getAngularPositionRad,
                shooterFeederMotorSim::getAngularVelocityRadPerSec
        );

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

        final Notifier simUpdateNotifier = new Notifier(() -> {
            final double dt = deltaTime.get();
            rightRollerSim.update(dt);
            leftRollerSim.update(dt);
            shooterFeederRollerSim.update(dt);
        });
        ToClose.add(simUpdateNotifier);
        simUpdateNotifier.setName(String.format(
                "SimUpdate(%d,%d,%d)",
                rightRoller.getDeviceID(),
                leftRoller.getDeviceID(),
                shooterFeederRoller.getDeviceID()
        ));
        simUpdateNotifier.startPeriodic(SIM_UPDATE_PERIOD_SEC);
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
        final InvertedValue rightInvertedValue = InvertedValue.CounterClockwise_Positive;
        final TalonFXConfiguration rightRollerConfig = new TalonFXConfiguration();
        rightRollerConfig.Slot0 = new Slot0Configs()
                .withKS(0)
                .withKV(0.18121)
                .withKA(2.9787)
                .withKP(3.6111);
        rightRollerConfig.TorqueCurrent.PeakForwardTorqueCurrent = 80;
        rightRollerConfig.TorqueCurrent.PeakReverseTorqueCurrent = -80;
        rightRollerConfig.CurrentLimits.StatorCurrentLimit = 60;
        rightRollerConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        rightRollerConfig.CurrentLimits.SupplyCurrentLimit = 40;
        rightRollerConfig.CurrentLimits.SupplyCurrentThreshold = 55;
        rightRollerConfig.CurrentLimits.SupplyTimeThreshold = 1.5;
        rightRollerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        rightRollerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        rightRollerConfig.MotorOutput.Inverted = rightInvertedValue;
        rightRollerConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        rightRollerConfig.Feedback.SensorToMechanismRatio = intakeConstants.rightMotorGearing();
        rightRoller.getConfigurator().apply(rightRollerConfig);

        final InvertedValue leftInvertedValue = InvertedValue.CounterClockwise_Positive;
        final TalonFXConfiguration leftRollerConfig = new TalonFXConfiguration();
        leftRollerConfig.Slot0 = new Slot0Configs()
                .withKS(0)
                .withKV(0.19557)
                .withKA(2.9856)
                .withKP(1.955);
        leftRollerConfig.TorqueCurrent.PeakForwardTorqueCurrent = 80;
        leftRollerConfig.TorqueCurrent.PeakReverseTorqueCurrent = -80;
        leftRollerConfig.CurrentLimits.StatorCurrentLimit = 60;
        leftRollerConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        leftRollerConfig.CurrentLimits.SupplyCurrentLimit = 40;
        leftRollerConfig.CurrentLimits.SupplyCurrentThreshold = 55;
        leftRollerConfig.CurrentLimits.SupplyTimeThreshold = 1.5;
        leftRollerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        leftRollerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        leftRollerConfig.MotorOutput.Inverted = leftInvertedValue;
        leftRollerConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        leftRollerConfig.Feedback.SensorToMechanismRatio = intakeConstants.leftMotorGearing();
        leftRoller.getConfigurator().apply(leftRollerConfig);

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

        SimUtils.setCTRETalonFXSimStateMotorInverted(rightRoller, rightInvertedValue);
        SimUtils.setCTRETalonFXSimStateMotorInverted(leftRoller, leftInvertedValue);
        SimUtils.setCTRETalonFXSimStateMotorInverted(shooterFeederRoller, shooterFeederInvertedValue);
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
            final double rightRollerTorqueCurrentAmp,
            final double leftRollerTorqueCurrentAmp,
            final double shooterFeederRollerTorqueCurrentAmp
    ) {
        rightRoller.setControl(torqueCurrentFOC.withOutput(rightRollerTorqueCurrentAmp));
        leftRoller.setControl(torqueCurrentFOC.withOutput(leftRollerTorqueCurrentAmp));
        shooterFeederRoller.setControl(torqueCurrentFOC.withOutput(shooterFeederRollerTorqueCurrentAmp));
    }

    @Override
    public void toVoltage(double rightRollerVolts, double leftRollerVolts, double shooterFeederRollerVolts) {
        rightRoller.setControl(voltageOut.withOutput(rightRollerVolts));
        leftRoller.setControl(voltageOut.withOutput(leftRollerVolts));
        shooterFeederRoller.setControl(voltageOut.withOutput(shooterFeederRollerVolts));
    }
}
