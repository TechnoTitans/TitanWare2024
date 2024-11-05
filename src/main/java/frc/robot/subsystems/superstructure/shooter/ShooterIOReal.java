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
import frc.robot.constants.HardwareConstants;

public class ShooterIOReal implements ShooterIO {
    private final HardwareConstants.ShooterConstants shooterConstants;

    private final TalonFX ampMotor;
    private final TalonFX leftFlywheelMotor;
    private final TalonFX rightFlywheelMotor;

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

    public ShooterIOReal(final HardwareConstants.ShooterConstants shooterConstants) {
        this.shooterConstants = shooterConstants;
        this.ampMotor = new TalonFX(shooterConstants.ampMotorId(), shooterConstants.CANBus());
        this.leftFlywheelMotor = new TalonFX(shooterConstants.leftFlywheelMotorId(), shooterConstants.CANBus());
        this.rightFlywheelMotor = new TalonFX(shooterConstants.rightFlywheelMotorId(), shooterConstants.CANBus());

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
    }

    @SuppressWarnings("DuplicatedCode")
    @Override
    public void config() {
        final TalonFXConfiguration ampTalonFXConfiguration = new TalonFXConfiguration();
        ampTalonFXConfiguration.Slot0 = new Slot0Configs()
                .withKS(5.6462)
                .withKV(0)
                .withKA(0.18083)
                .withKP(3.5468);
        ampTalonFXConfiguration.TorqueCurrent.PeakForwardTorqueCurrent = 80;
        ampTalonFXConfiguration.TorqueCurrent.PeakReverseTorqueCurrent = -80;
        ampTalonFXConfiguration.CurrentLimits.StatorCurrentLimit = 60;
        ampTalonFXConfiguration.CurrentLimits.StatorCurrentLimitEnable = true;
        ampTalonFXConfiguration.CurrentLimits.SupplyCurrentLimit = 40;
        ampTalonFXConfiguration.CurrentLimits.SupplyCurrentThreshold = 60;
        ampTalonFXConfiguration.CurrentLimits.SupplyTimeThreshold = 1.5;
        ampTalonFXConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;
        ampTalonFXConfiguration.Feedback.SensorToMechanismRatio = shooterConstants.ampMotorGearing();
        ampTalonFXConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        ampTalonFXConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        ampMotor.getConfigurator().apply(ampTalonFXConfiguration);

        final TalonFXConfiguration leftTalonFXConfiguration = new TalonFXConfiguration();
        leftTalonFXConfiguration.Slot0 = new Slot0Configs()
                .withKS(10.669)
                .withKV(0.1978)
                .withKA(0.40341)
                .withKP(5);
        leftTalonFXConfiguration.TorqueCurrent.PeakForwardTorqueCurrent = 60;
        leftTalonFXConfiguration.TorqueCurrent.PeakReverseTorqueCurrent = -60;
        leftTalonFXConfiguration.CurrentLimits.StatorCurrentLimit = 60;
        leftTalonFXConfiguration.CurrentLimits.StatorCurrentLimitEnable = true;
        leftTalonFXConfiguration.CurrentLimits.SupplyCurrentLimit = 40;
        leftTalonFXConfiguration.CurrentLimits.SupplyCurrentThreshold = 50;
        leftTalonFXConfiguration.CurrentLimits.SupplyTimeThreshold = 0.5;
        leftTalonFXConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;
        leftTalonFXConfiguration.Feedback.SensorToMechanismRatio = shooterConstants.leftFlywheelGearing();
        leftTalonFXConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        leftTalonFXConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        leftFlywheelMotor.getConfigurator().apply(leftTalonFXConfiguration);

        final TalonFXConfiguration rightTalonFXConfiguration = new TalonFXConfiguration();
        rightTalonFXConfiguration.Slot0 = new Slot0Configs()
                .withKS(10.712)
                .withKV(0.15193)
                .withKA(0.43766)
                .withKP(5);
        rightTalonFXConfiguration.TorqueCurrent.PeakForwardTorqueCurrent = 60;
        rightTalonFXConfiguration.TorqueCurrent.PeakReverseTorqueCurrent = -60;
        rightTalonFXConfiguration.CurrentLimits.StatorCurrentLimit = 60;
        rightTalonFXConfiguration.CurrentLimits.StatorCurrentLimitEnable = true;
        rightTalonFXConfiguration.CurrentLimits.SupplyCurrentLimit = 40;
        rightTalonFXConfiguration.CurrentLimits.SupplyCurrentThreshold = 50;
        rightTalonFXConfiguration.CurrentLimits.SupplyTimeThreshold = 0.5;
        rightTalonFXConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;
        rightTalonFXConfiguration.Feedback.SensorToMechanismRatio = shooterConstants.rightFlywheelGearing();
        rightTalonFXConfiguration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        rightTalonFXConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        rightFlywheelMotor.getConfigurator().apply(rightTalonFXConfiguration);

        // TODO: we need to be smart about what frequency we set to these, since if we're running SysId, they should
        //  be fast, but if not, they can be slower & we care about the remaining signals
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
    }

    @SuppressWarnings("DuplicatedCode")
    @Override
    public void updateInputs(final ShooterIO.ShooterIOInputs inputs) {
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
        inputs.ampVoltageVolts = _ampVoltage.getValue();
        inputs.ampCurrentAmps = _ampTorqueCurrent.getValue();
        inputs.ampTempCelsius = _ampDeviceTemp.getValue();
        inputs.leftPositionRots = _leftPosition.getValue();
        inputs.leftVelocityRotsPerSec = _leftVelocity.getValue();
        inputs.leftVoltageVolts = _leftVoltage.getValue();
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
            final double desiredTopVelocity,
            final double desiredBottomVelocity
    ) {
        ampMotor.setControl(velocityTorqueCurrentFOC.withVelocity(ampVelocity));
        leftFlywheelMotor.setControl(velocityTorqueCurrentFOC.withVelocity(desiredTopVelocity));
        rightFlywheelMotor.setControl(velocityTorqueCurrentFOC.withVelocity(desiredBottomVelocity));
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
