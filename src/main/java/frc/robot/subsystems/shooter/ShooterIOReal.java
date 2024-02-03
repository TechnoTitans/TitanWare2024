package frc.robot.subsystems.shooter;

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
    private final TalonFX topMotor;
    private final TalonFX bottomMotor;

    private final VelocityTorqueCurrentFOC velocityTorqueCurrentFOC;
    private final TorqueCurrentFOC torqueCurrentFOC;
    private final VoltageOut voltageOut;

    // Cached StatusSignals
    private final StatusSignal<Double> _topPosition;
    private final StatusSignal<Double> _topVelocity;
    private final StatusSignal<Double> _topVoltage;
    private final StatusSignal<Double> _topTorqueCurrent;
    private final StatusSignal<Double> _topDeviceTemp;
    private final StatusSignal<Double> _bottomPosition;
    private final StatusSignal<Double> _bottomVelocity;
    private final StatusSignal<Double> _bottomVoltage;
    private final StatusSignal<Double> _bottomTorqueCurrent;
    private final StatusSignal<Double> _bottomDeviceTemp;

    public ShooterIOReal(final HardwareConstants.ShooterConstants shooterConstants) {
        this.topMotor = new TalonFX(shooterConstants.topMotorId(), shooterConstants.CANbus());
        this.bottomMotor = new TalonFX(shooterConstants.bottomMotorId(), shooterConstants.CANbus());

        this.velocityTorqueCurrentFOC = new VelocityTorqueCurrentFOC(0);
        this.torqueCurrentFOC = new TorqueCurrentFOC(0);
        this.voltageOut = new VoltageOut(0);

        this._topPosition = topMotor.getPosition();
        this._topVelocity = topMotor.getVelocity();
        this._topVoltage = topMotor.getMotorVoltage();
        this._topTorqueCurrent = topMotor.getTorqueCurrent();
        this._topDeviceTemp = topMotor.getDeviceTemp();
        this._bottomPosition = bottomMotor.getPosition();
        this._bottomVelocity = bottomMotor.getVelocity();
        this._bottomVoltage = bottomMotor.getMotorVoltage();
        this._bottomTorqueCurrent = bottomMotor.getTorqueCurrent();
        this._bottomDeviceTemp = bottomMotor.getDeviceTemp();
    }

    @SuppressWarnings("DuplicatedCode")
    @Override
    public void config() {
        final TalonFXConfiguration topTalonFXConfiguration = new TalonFXConfiguration();
        topTalonFXConfiguration.Slot0 = new Slot0Configs()
                .withKS(2.4818)
                .withKV(0.20086)
                .withKA(0.55503)
                .withKP(2.5941);
        topTalonFXConfiguration.TorqueCurrent.PeakForwardTorqueCurrent = 80;
        topTalonFXConfiguration.TorqueCurrent.PeakReverseTorqueCurrent = -80;
        topTalonFXConfiguration.CurrentLimits.StatorCurrentLimit = 80;
        topTalonFXConfiguration.CurrentLimits.StatorCurrentLimitEnable = true;
        topTalonFXConfiguration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        topTalonFXConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        topMotor.getConfigurator().apply(topTalonFXConfiguration);

        final TalonFXConfiguration bottomTalonFXConfiguration = new TalonFXConfiguration();
        bottomTalonFXConfiguration.Slot0 = new Slot0Configs()
                .withKS(2.4682)
                .withKV(0.099933)
                .withKA(0.20584)
                .withKP(1.2235);
        bottomTalonFXConfiguration.CurrentLimits.StatorCurrentLimit = 80;
        bottomTalonFXConfiguration.CurrentLimits.StatorCurrentLimitEnable = true;
        bottomTalonFXConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        bottomTalonFXConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        bottomMotor.getConfigurator().apply(bottomTalonFXConfiguration);

        // TODO: we need to be smart about what frequency we set to these, since if we're running SysId, they should
        //  be fast, but if not, they can be slower & we care about the remaining signals
        BaseStatusSignal.setUpdateFrequencyForAll(
                250,
                _topPosition,
                _topVelocity,
                _topVoltage,
                _bottomPosition,
                _bottomVelocity,
                _bottomVoltage
        );
        BaseStatusSignal.setUpdateFrequencyForAll(
                50,
                _topTorqueCurrent,
                _topDeviceTemp,
                _bottomTorqueCurrent,
                _bottomDeviceTemp
        );
        ParentDevice.optimizeBusUtilizationForAll(
                topMotor,
                bottomMotor
        );
    }

    @SuppressWarnings("DuplicatedCode")
    @Override
    public void updateInputs(final ShooterIO.ShooterIOInputs inputs) {
        BaseStatusSignal.refreshAll(
                _topPosition,
                _topVelocity,
                _topTorqueCurrent,
                _topDeviceTemp,
                _bottomPosition,
                _bottomVelocity,
                _bottomTorqueCurrent,
                _bottomDeviceTemp
        );

        inputs.topPositionRots = _topPosition.getValue();
        inputs.topVelocityRotsPerSec = _topVelocity.getValue();
        inputs.topCurrentAmps = _topTorqueCurrent.getValue();
        inputs.topTempCelsius = _topDeviceTemp.getValue();
        inputs.bottomPositionRots = _bottomPosition.getValue();
        inputs.bottomVelocityRotsPerSec = _bottomVelocity.getValue();
        inputs.bottomCurrentAmps = _bottomTorqueCurrent.getValue();
        inputs.bottomTempCelsius = _bottomDeviceTemp.getValue();
    }

    @Override
    public void setInputs(final double desiredTopVelocity, final double desiredBottomVelocity) {
        topMotor.setControl(velocityTorqueCurrentFOC.withVelocity(desiredTopVelocity));
        bottomMotor.setControl(velocityTorqueCurrentFOC.withVelocity(desiredBottomVelocity));
    }

    @Override
    public void setCharacterizationVolts(double topVolts, double bottomVolts) {
        topMotor.setControl(voltageOut.withOutput(topVolts));
        bottomMotor.setControl(voltageOut.withOutput(bottomVolts));
    }

    @Override
    public void setCharacterizationTorqueCurrent(double topTorqueCurrentAmps, double bottomTorqueCurrentAmps) {
        topMotor.setControl(torqueCurrentFOC.withOutput(topTorqueCurrentAmps));
        bottomMotor.setControl(torqueCurrentFOC.withOutput(bottomTorqueCurrentAmps));
    }
}
