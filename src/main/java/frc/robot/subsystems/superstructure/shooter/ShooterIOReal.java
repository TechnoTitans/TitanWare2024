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

    private final TalonFX leftFlywheelMotor;
    private final TalonFX rightFlywheelMotor;

    private final VelocityTorqueCurrentFOC velocityTorqueCurrentFOC;
    private final TorqueCurrentFOC torqueCurrentFOC;
    private final VoltageOut voltageOut;

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

    public ShooterIOReal(final HardwareConstants.ShooterConstants shooterConstants) {
        this.shooterConstants = shooterConstants;
        this.leftFlywheelMotor = new TalonFX(shooterConstants.leftFlywheelMotorId(), shooterConstants.CANBus());
        this.rightFlywheelMotor = new TalonFX(shooterConstants.rightFlywheelMotorId(), shooterConstants.CANBus());

        this.velocityTorqueCurrentFOC = new VelocityTorqueCurrentFOC(0);
        this.torqueCurrentFOC = new TorqueCurrentFOC(0);
        this.voltageOut = new VoltageOut(0);

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
        final TalonFXConfiguration leftTalonFXConfiguration = new TalonFXConfiguration();
        leftTalonFXConfiguration.Slot0 = new Slot0Configs()
                .withKS(2.4818)
                .withKV(0.20086)
                .withKA(0.55503)
                .withKP(2.5941);
        leftTalonFXConfiguration.TorqueCurrent.PeakForwardTorqueCurrent = 80;
        leftTalonFXConfiguration.TorqueCurrent.PeakReverseTorqueCurrent = -80;
        leftTalonFXConfiguration.CurrentLimits.StatorCurrentLimit = 80;
        leftTalonFXConfiguration.CurrentLimits.StatorCurrentLimitEnable = true;
        leftTalonFXConfiguration.Feedback.SensorToMechanismRatio = shooterConstants.leftFlywheelGearing();
        leftTalonFXConfiguration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        leftTalonFXConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        leftFlywheelMotor.getConfigurator().apply(leftTalonFXConfiguration);

        final TalonFXConfiguration rightTalonFXConfiguration = new TalonFXConfiguration();
        rightTalonFXConfiguration.Slot0 = new Slot0Configs()
                .withKS(2.4682)
                .withKV(0.099933)
                .withKA(0.20584)
                .withKP(1.2235);
        rightTalonFXConfiguration.TorqueCurrent.PeakForwardTorqueCurrent = 80;
        rightTalonFXConfiguration.TorqueCurrent.PeakReverseTorqueCurrent = -80;
        rightTalonFXConfiguration.CurrentLimits.StatorCurrentLimit = 80;
        rightTalonFXConfiguration.CurrentLimits.StatorCurrentLimitEnable = true;
        leftTalonFXConfiguration.Feedback.SensorToMechanismRatio = shooterConstants.rightFlywheelGearing();
        rightTalonFXConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        rightTalonFXConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        rightFlywheelMotor.getConfigurator().apply(rightTalonFXConfiguration);

        // TODO: we need to be smart about what frequency we set to these, since if we're running SysId, they should
        //  be fast, but if not, they can be slower & we care about the remaining signals
        BaseStatusSignal.setUpdateFrequencyForAll(
                100,
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
                leftFlywheelMotor,
                rightFlywheelMotor
        );
    }

    @SuppressWarnings("DuplicatedCode")
    @Override
    public void updateInputs(final ShooterIO.ShooterIOInputs inputs) {
        BaseStatusSignal.refreshAll(
                _leftPosition,
                _leftVelocity,
                _leftTorqueCurrent,
                _leftDeviceTemp,
                _rightPosition,
                _rightVelocity,
                _rightTorqueCurrent,
                _rightDeviceTemp
        );

        inputs.leftPositionRots = _leftPosition.getValue();
        inputs.leftVelocityRotsPerSec = _leftVelocity.getValue();
        inputs.leftCurrentAmps = _leftTorqueCurrent.getValue();
        inputs.leftTempCelsius = _leftDeviceTemp.getValue();
        inputs.rightPositionRots = _rightPosition.getValue();
        inputs.rightVelocityRotsPerSec = _rightVelocity.getValue();
        inputs.rightCurrentAmps = _rightTorqueCurrent.getValue();
        inputs.rightTempCelsius = _rightDeviceTemp.getValue();
    }

    @Override
    public void toVelocity(final double desiredTopVelocity, final double desiredBottomVelocity) {
        leftFlywheelMotor.setControl(velocityTorqueCurrentFOC.withVelocity(desiredTopVelocity));
        rightFlywheelMotor.setControl(velocityTorqueCurrentFOC.withVelocity(desiredBottomVelocity));
    }

    @Override
    public void setCharacterizationVolts(final double leftVolts, final double rightVolts) {
        leftFlywheelMotor.setControl(voltageOut.withOutput(leftVolts));
        rightFlywheelMotor.setControl(voltageOut.withOutput(rightVolts));
    }

    @Override
    public void setCharacterizationTorqueCurrent(
            final double leftTorqueCurrentAmps,
            final double rightTorqueCurrentAmps
    ) {
        leftFlywheelMotor.setControl(torqueCurrentFOC.withOutput(leftTorqueCurrentAmps));
        rightFlywheelMotor.setControl(torqueCurrentFOC.withOutput(rightTorqueCurrentAmps));
    }
}
