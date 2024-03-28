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
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.constants.HardwareConstants;
import frc.robot.utils.closeables.ToClose;
import frc.robot.utils.control.DeltaTime;
import frc.robot.utils.sim.SimUtils;
import frc.robot.utils.sim.motors.TalonFXSim;

public class ClimbIOSim implements ClimbIO {
    private static final double SIM_UPDATE_PERIOD_SEC = 0.005;

    private final HardwareConstants.ClimbConstants climbConstants;
    private final DeltaTime deltaTime;

    private final TalonFX rightArm;
    private final TalonFX leftArm;

    private final TalonFXSim rightArmSim;
    private final TalonFXSim leftArmSim;

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

    public ClimbIOSim(final HardwareConstants.ClimbConstants climbConstants) {
        this.climbConstants = climbConstants;
        this.deltaTime = new DeltaTime(true);

        this.rightArm = new TalonFX(climbConstants.rightArmMotorId(), climbConstants.CANBus());
        this.leftArm = new TalonFX(climbConstants.leftArmMotorId(), climbConstants.CANBus());

        final DCMotorSim rightArmMotorSim = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(
                        //TODO: Get Correct Values
                        0.18121 / (2 * Math.PI),
                        2.9787 / (2 * Math.PI)
                ),
                DCMotor.getKrakenX60Foc(1),
                climbConstants.climbGearing()
        );

        this.rightArmSim = new TalonFXSim(
                rightArm,
                climbConstants.climbGearing(),
                rightArmMotorSim::update,
                rightArmMotorSim::setInputVoltage,
                rightArmMotorSim::getAngularPositionRad,
                rightArmMotorSim::getAngularVelocityRadPerSec
        );

        final DCMotorSim leftArmMotorSim = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(
                        //TODO: Get Correct Values
                        0.18121 / (2 * Math.PI),
                        2.9787 / (2 * Math.PI)
                ),
                DCMotor.getKrakenX60Foc(1),
                climbConstants.climbGearing()
        );

        this.leftArmSim = new TalonFXSim(
                rightArm,
                climbConstants.climbGearing(),
                leftArmMotorSim::update,
                leftArmMotorSim::setInputVoltage,
                leftArmMotorSim::getAngularPositionRad,
                leftArmMotorSim::getAngularVelocityRadPerSec
        );

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

        final Notifier simUpdateNotifier = new Notifier(() -> {
            final double dt = deltaTime.get();
            rightArmSim.update(dt);
            leftArmSim.update(dt);
        });
        ToClose.add(simUpdateNotifier);
        simUpdateNotifier.setName(String.format(
                "SimUpdate(%d,%d)",
                rightArm.getDeviceID(),
                leftArm.getDeviceID()
        ));
        simUpdateNotifier.startPeriodic(SIM_UPDATE_PERIOD_SEC);
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
        final InvertedValue rightArmInvertedValue = InvertedValue.CounterClockwise_Positive;
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
        rightArmConfig.MotorOutput.Inverted = rightArmInvertedValue;
        rightArmConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        rightArmConfig.Feedback.SensorToMechanismRatio = climbConstants.climbGearing();
        rightArm.getConfigurator().apply(rightArmConfig);

        final InvertedValue leftArmInvertedValue = InvertedValue.CounterClockwise_Positive;
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
        leftArmConfig.MotorOutput.Inverted = leftArmInvertedValue;
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

        SimUtils.setCTRETalonFXSimStateMotorInverted(rightArm, rightArmInvertedValue);
        SimUtils.setCTRETalonFXSimStateMotorInverted(leftArm, leftArmInvertedValue);
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
