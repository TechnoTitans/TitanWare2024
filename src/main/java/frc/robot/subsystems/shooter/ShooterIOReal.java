package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import frc.robot.constants.HardwareConstants;

public class ShooterIOReal implements ShooterIO {
    private final TalonFX topMotor;
    private final TalonFX bottomMotor;

    private final DutyCycleOut dutyCycleOut;
    private final VoltageOut voltageOut;

    private final BangBangController topBangBangController;
    private final BangBangController bottomBangBangController;

    private final SimpleMotorFeedforward topFF;
    private final SimpleMotorFeedforward bottomFF;

    // Cached StatusSignals
    private final StatusSignal<Double> _topPosition;
    private final StatusSignal<Double> _topVelocity;
    private final StatusSignal<Double> _topVoltage;
    private final StatusSignal<Double> _topTorqueCurrent;
    private final StatusSignal<Double> _topDutyCycle;
    private final StatusSignal<Double> _topDeviceTemp;
    private final StatusSignal<Double> _bottomPosition;
    private final StatusSignal<Double> _bottomVelocity;
    private final StatusSignal<Double> _bottomVoltage;
    private final StatusSignal<Double> _bottomTorqueCurrent;
    private final StatusSignal<Double> _bottomDutyCycle;
    private final StatusSignal<Double> _bottomDeviceTemp;

    public ShooterIOReal(final HardwareConstants.ShooterConstants shooterConstants) {
        this.topMotor = new TalonFX(shooterConstants.topMotorId(), shooterConstants.CANbus());
        this.bottomMotor = new TalonFX(shooterConstants.bottomMotorId(), shooterConstants.CANbus());

        this.dutyCycleOut = new DutyCycleOut(0);
        this.voltageOut = new VoltageOut(0);

        this.topBangBangController = new BangBangController(10);
        this.bottomBangBangController = new BangBangController(10);

        this.topFF = new SimpleMotorFeedforward(0.094, 0.121, 0.01);
        this.bottomFF = new SimpleMotorFeedforward(0.11, 0.122, 0.011);

        this._topPosition = topMotor.getPosition();
        this._topVelocity = topMotor.getVelocity();
        this._topVoltage = topMotor.getMotorVoltage();
        this._topTorqueCurrent = topMotor.getTorqueCurrent();
        this._topDutyCycle = topMotor.getDutyCycle();
        this._topDeviceTemp = topMotor.getDeviceTemp();
        this._bottomPosition = bottomMotor.getPosition();
        this._bottomVelocity = bottomMotor.getVelocity();
        this._bottomVoltage = bottomMotor.getMotorVoltage();
        this._bottomTorqueCurrent = bottomMotor.getTorqueCurrent();
        this._bottomDutyCycle = bottomMotor.getDutyCycle();
        this._bottomDeviceTemp = bottomMotor.getDeviceTemp();
    }

    @SuppressWarnings("DuplicatedCode")
    @Override
    public void config() {
        final TalonFXConfiguration topTalonFXConfiguration = new TalonFXConfiguration();
        topTalonFXConfiguration.CurrentLimits.StatorCurrentLimit = 80;
        topTalonFXConfiguration.CurrentLimits.StatorCurrentLimitEnable = true;
        topTalonFXConfiguration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        topTalonFXConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        topMotor.getConfigurator().apply(topTalonFXConfiguration);

        final TalonFXConfiguration bottomTalonFXConfiguration = new TalonFXConfiguration();
        bottomTalonFXConfiguration.CurrentLimits.StatorCurrentLimit = 80;
        bottomTalonFXConfiguration.CurrentLimits.StatorCurrentLimitEnable = true;
        bottomTalonFXConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        bottomTalonFXConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        bottomMotor.getConfigurator().apply(bottomTalonFXConfiguration);

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
                5,
                _topTorqueCurrent,
                _topDutyCycle,
                _topDeviceTemp,
                _bottomTorqueCurrent,
                _bottomDutyCycle,
                _bottomDeviceTemp
        );
        ParentDevice.optimizeBusUtilizationForAll(
                topMotor,
                bottomMotor
        );
    }

    @Override
    public void updateInputs(final ShooterIO.ShooterIOInputs inputs) {
        BaseStatusSignal.refreshAll(
                _topPosition,
                _topVelocity,
                _topTorqueCurrent,
                _topDutyCycle,
                _topDeviceTemp,
                _bottomPosition,
                _bottomVelocity,
                _bottomTorqueCurrent,
                _bottomDutyCycle,
                _bottomDeviceTemp
        );

        inputs.topPositionRots = _topPosition.getValue();
        inputs.topVelocityRotsPerSec = _topVelocity.getValue();
        inputs.topPercentOutput = _topDutyCycle.getValue();
        inputs.topCurrentAmps = _topTorqueCurrent.getValue();
        inputs.topTempCelsius = _topDeviceTemp.getValue();
        inputs.bottomPositionRots = _bottomPosition.getValue();
        inputs.bottomVelocityRotsPerSec = _bottomVelocity.getValue();
        inputs.bottomPercentOutput = _bottomDutyCycle.getValue();
        inputs.bottomCurrentAmps = _bottomTorqueCurrent.getValue();
        inputs.bottomTempCelsius = _bottomDeviceTemp.getValue();
    }

    @Override
    public void setInputs(final double desiredTopVelocity, final double desiredBottomVelocity) {
//        topMotor.setControl(dutyCycleOut.withOutput(
//                topBangBangController.calculate(_topVelocity.getValue(), desiredTopVelocity)
//        ));
//        bottomMotor.setControl(dutyCycleOut.withOutput(
//                bottomBangBangController.calculate(_bottomVelocity.getValue(), desiredBottomVelocity)
//        ));
        topMotor.setControl(voltageOut.withOutput(topFF.calculate(desiredTopVelocity)));
        bottomMotor.setControl(voltageOut.withOutput(bottomFF.calculate(desiredBottomVelocity)));
    }

    @Override
    public void setCharacterizationVolts(double topVolts, double bottomVolts) {
        topMotor.setControl(voltageOut.withOutput(topVolts));
        bottomMotor.setControl(voltageOut.withOutput(bottomVolts));
    }
}
