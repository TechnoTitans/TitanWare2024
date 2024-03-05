package frc.robot.subsystems.superstructure.arm;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.simulation.DIOSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.constants.HardwareConstants;
import frc.robot.utils.closeables.ToClose;
import frc.robot.utils.control.DeltaTime;
import frc.robot.utils.ctre.Phoenix6Utils;
import frc.robot.utils.sim.SimUtils;
import frc.robot.utils.sim.motors.TalonFXSim;

import java.util.List;

public class ArmIOSim implements ArmIO {
    private static final double SIM_UPDATE_PERIOD_SEC = 0.005;

    private final DeltaTime deltaTime;
    private final HardwareConstants.ArmConstants armConstants;

    private final SingleJointedArmSim armPivotSim;

    private final TalonFX leftPivotMotor;
    private final TalonFX rightPivotMotor;
    private final TalonFXSim pivotMotorsSim;

    private final DigitalInput pivotLowerLimitSwitch;
    private final DIOSim pivotLowerLimitSwitchSim;

    private final MotionMagicExpoVoltage motionMagicExpoVoltage;
    private final VoltageOut voltageOut;
    private final TorqueCurrentFOC torqueCurrentFOC;

    // Cached StatusSignals
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

    public ArmIOSim(final HardwareConstants.ArmConstants armConstants) {
        this.deltaTime = new DeltaTime(true);
        this.armConstants = armConstants;

        final double pivotLowerLimitsRots = armConstants.pivotSoftLowerLimitRots();
        this.armPivotSim = new SingleJointedArmSim(
                LinearSystemId.identifyPositionSystem(
                        13.97 / (2d * Math.PI),
                        0.03 / (2d * Math.PI)
                ),
                DCMotor.getKrakenX60Foc(2),
                armConstants.pivotGearing(),
                0.508,
                0,
                Math.PI,
                true,
                Units.rotationsToRadians(pivotLowerLimitsRots)
        );

        this.leftPivotMotor = new TalonFX(armConstants.leftPivotMotorId(), armConstants.CANBus());
        this.rightPivotMotor = new TalonFX(armConstants.rightPivotMotorId(), armConstants.CANBus());
        this.pivotMotorsSim = new TalonFXSim(
                List.of(leftPivotMotor, rightPivotMotor),
                armConstants.pivotGearing(),
                armPivotSim::update,
                armPivotSim::setInputVoltage,
                armPivotSim::getAngleRads,
                armPivotSim::getVelocityRadPerSec
        );

        this.pivotLowerLimitSwitch = new DigitalInput(armConstants.pivotZeroingSwitchDIOChannel());
        this.pivotLowerLimitSwitchSim = new DIOSim(pivotLowerLimitSwitch);
        this.pivotLowerLimitSwitchSim.setInitialized(true);
        this.pivotLowerLimitSwitchSim.setIsInput(true);
        this.pivotLowerLimitSwitchSim.setValue(false);

        this.motionMagicExpoVoltage = new MotionMagicExpoVoltage(0);
        this.voltageOut = new VoltageOut(0);
        this.torqueCurrentFOC = new TorqueCurrentFOC(0);

        this._leftPosition = leftPivotMotor.getPosition();
        this._leftVelocity = leftPivotMotor.getVelocity();
        this._leftVoltage = leftPivotMotor.getMotorVoltage();
        this._leftTorqueCurrent = leftPivotMotor.getTorqueCurrent();
        this._leftDeviceTemp = leftPivotMotor.getDeviceTemp();
        this._rightPosition = rightPivotMotor.getPosition();
        this._rightVelocity = rightPivotMotor.getVelocity();
        this._rightVoltage = rightPivotMotor.getMotorVoltage();
        this._rightTorqueCurrent = rightPivotMotor.getTorqueCurrent();
        this._rightDeviceTemp = rightPivotMotor.getDeviceTemp();

        final Notifier simUpdateNotifier = new Notifier(() -> {
            final double dt = deltaTime.get();
            pivotMotorsSim.update(dt);
            pivotLowerLimitSwitchSim.setValue(
                    _leftPosition.refresh().getValue() <= pivotLowerLimitsRots
                            || _rightPosition.refresh().getValue() <= pivotLowerLimitsRots
            );
        });
        ToClose.add(simUpdateNotifier);
        simUpdateNotifier.setName(String.format(
                "SimUpdate(%d,%d)",
                leftPivotMotor.getDeviceID(),
                rightPivotMotor.getDeviceID()
        ));
        simUpdateNotifier.startPeriodic(SIM_UPDATE_PERIOD_SEC);
    }

    @SuppressWarnings("DuplicatedCode")
    @Override
    public void config(
            final Arm.PositionSetpoint pivotSoftLowerLimit,
            final Arm.PositionSetpoint pivotSoftUpperLimit
    ) {
        final TalonFXConfiguration leftTalonFXConfiguration = new TalonFXConfiguration();
        final InvertedValue leftTalonFXInverted = InvertedValue.Clockwise_Positive;
        leftTalonFXConfiguration.Slot0 = new Slot0Configs()
                .withKS(0)
                .withKG(0.11)
                .withGravityType(GravityTypeValue.Arm_Cosine)
                .withKV(13.97)
                .withKA(0.015)
                .withKP(50); // TODO: tune Kp
        leftTalonFXConfiguration.MotionMagic.MotionMagicCruiseVelocity = 0;
        leftTalonFXConfiguration.MotionMagic.MotionMagicExpo_kV = 13.97;
        leftTalonFXConfiguration.MotionMagic.MotionMagicExpo_kA = 0.015;
        leftTalonFXConfiguration.CurrentLimits.StatorCurrentLimit = 60;
        leftTalonFXConfiguration.CurrentLimits.StatorCurrentLimitEnable = true;
        leftTalonFXConfiguration.Feedback.SensorToMechanismRatio = armConstants.pivotGearing();
        leftTalonFXConfiguration.SoftwareLimitSwitch.ReverseSoftLimitThreshold = pivotSoftLowerLimit.pivotPositionRots;
        leftTalonFXConfiguration.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        leftTalonFXConfiguration.SoftwareLimitSwitch.ForwardSoftLimitThreshold = pivotSoftUpperLimit.pivotPositionRots;
        leftTalonFXConfiguration.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        leftTalonFXConfiguration.MotorOutput.Inverted = leftTalonFXInverted;
        leftTalonFXConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        leftPivotMotor.getConfigurator().apply(leftTalonFXConfiguration);

        final TalonFXConfiguration rightTalonFXConfiguration = new TalonFXConfiguration();
        final InvertedValue rightTalonFXInverted = InvertedValue.CounterClockwise_Positive;
        rightTalonFXConfiguration.Slot0 = new Slot0Configs()
                .withKS(0)
                .withKG(0.11)
                .withGravityType(GravityTypeValue.Arm_Cosine)
                .withKV(13.97)
                .withKA(0.015)
                .withKP(50); // TODO: tune Kp
        rightTalonFXConfiguration.MotionMagic.MotionMagicCruiseVelocity = 0;
        rightTalonFXConfiguration.MotionMagic.MotionMagicExpo_kV = 13.97;
        rightTalonFXConfiguration.MotionMagic.MotionMagicExpo_kA = 0.015;
        rightTalonFXConfiguration.CurrentLimits.StatorCurrentLimit = 60;
        rightTalonFXConfiguration.CurrentLimits.StatorCurrentLimitEnable = true;
        rightTalonFXConfiguration.Feedback.SensorToMechanismRatio = armConstants.pivotGearing();
        rightTalonFXConfiguration.SoftwareLimitSwitch.ReverseSoftLimitThreshold = pivotSoftLowerLimit.pivotPositionRots;
        rightTalonFXConfiguration.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        rightTalonFXConfiguration.SoftwareLimitSwitch.ForwardSoftLimitThreshold = pivotSoftUpperLimit.pivotPositionRots;
        rightTalonFXConfiguration.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        rightTalonFXConfiguration.MotorOutput.Inverted = rightTalonFXInverted;
        rightTalonFXConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        rightPivotMotor.getConfigurator().apply(rightTalonFXConfiguration);

        BaseStatusSignal.setUpdateFrequencyForAll(
                250,
                _leftPosition,
                _leftVelocity,
                _leftVoltage,
                _leftTorqueCurrent,
                _rightPosition,
                _rightVelocity,
                _rightVoltage,
                _rightTorqueCurrent
        );
        BaseStatusSignal.setUpdateFrequencyForAll(
                4,
                _leftDeviceTemp,
                _rightDeviceTemp
        );
        ParentDevice.optimizeBusUtilizationForAll(
                leftPivotMotor,
                rightPivotMotor
        );

        SimUtils.setCTRETalonFXSimStateMotorInverted(leftPivotMotor, leftTalonFXInverted);
        SimUtils.setCTRETalonFXSimStateMotorInverted(rightPivotMotor, rightTalonFXInverted);
    }

    @SuppressWarnings("DuplicatedCode")
    @Override
    public void updateInputs(final ArmIOInputs inputs) {
        BaseStatusSignal.refreshAll(
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

        inputs.pivotLowerLimitSwitch = pivotLowerLimitSwitch.get();

        inputs.leftPivotPositionRots = _leftPosition.getValue();
        inputs.leftPivotVelocityRotsPerSec = _leftVelocity.getValue();
        inputs.leftPivotVoltageVolts = _leftVoltage.getValue();
        inputs.leftPivotTorqueCurrentAmps = _leftTorqueCurrent.getValue();
        inputs.leftPivotTempCelsius = _leftDeviceTemp.getValue();

        inputs.rightPivotPositionRots = _rightPosition.getValue();
        inputs.rightPivotVelocityRotsPerSec = _rightVelocity.getValue();
        inputs.rightPivotVoltageVolts = _rightVoltage.getValue();
        inputs.rightPivotTorqueCurrentAmps = _rightTorqueCurrent.getValue();
        inputs.rightPivotTempCelsius = _rightDeviceTemp.getValue();
    }

    @Override
    public void setPivotPosition(final double pivotPositionRots) {
        // TODO: this should probably retry it if it fails in addition to just reporting it
        Phoenix6Utils.reportIfNotOk(leftPivotMotor, leftPivotMotor.setPosition(pivotPositionRots));
        Phoenix6Utils.reportIfNotOk(rightPivotMotor, rightPivotMotor.setPosition(pivotPositionRots));
    }

    @Override
    public void toPivotPosition(final double pivotPositionRots) {
        leftPivotMotor.setControl(motionMagicExpoVoltage.withPosition(pivotPositionRots));
        rightPivotMotor.setControl(motionMagicExpoVoltage.withPosition(pivotPositionRots));
    }

    @Override
    public void toPivotVoltage(final double pivotVolts) {
        leftPivotMotor.setControl(voltageOut.withOutput(pivotVolts));
        rightPivotMotor.setControl(voltageOut.withOutput(pivotVolts));
    }

    @Override
    public void toPivotTorqueCurrent(double pivotTorqueCurrentAmps) {
        leftPivotMotor.setControl(torqueCurrentFOC.withOutput(pivotTorqueCurrentAmps));
        rightPivotMotor.setControl(torqueCurrentFOC.withOutput(pivotTorqueCurrentAmps));
    }
}
