package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.constants.Constants;
import frc.robot.constants.HardwareConstants;
import frc.robot.utils.control.DeltaTime;
import frc.robot.utils.sim.SimUtils;
import frc.robot.utils.sim.motors.CTREPhoenix6TalonFXSim;
import org.littletonrobotics.junction.Logger;

public class ShooterIOSim implements ShooterIO {
    private final DeltaTime deltaTime;

    private final TalonFX topMotor;
    private final TalonFX bottomMotor;

    final TalonFXConfiguration topTalonFXConfiguration = new TalonFXConfiguration();
    final TalonFXConfiguration bottomTalonFXConfiguration = new TalonFXConfiguration();

    private final CTREPhoenix6TalonFXSim topMotorSim;
    private final CTREPhoenix6TalonFXSim bottomMotorSim;

    private final DutyCycleOut topDutyCycleOut;
    private final DutyCycleOut bottomDutyCycleOut;

    private final BangBangController topBangBangController;
    private final BangBangController bottomBangBangController;

    private double topVelocitySetpoint;
    private double bottomVelocitySetpoint;


    // Cached StatusSignals
    private final StatusSignal<Double> _topPosition;
    private final StatusSignal<Double> _topVelocity;
    private final StatusSignal<Double> _topTorqueCurrent;
    private final StatusSignal<Double> _topDutyCycle;
    private final StatusSignal<Double> _topDeviceTemp;
    private final StatusSignal<Double> _bottomPosition;
    private final StatusSignal<Double> _bottomVelocity;
    private final StatusSignal<Double> _bottomTorqueCurrent;
    private final StatusSignal<Double> _bottomDutyCycle;
    private final StatusSignal<Double> _bottomDeviceTemp;

    public ShooterIOSim(final HardwareConstants.ShooterConstants shooterConstants) {
        this.deltaTime = new DeltaTime();

        this.topMotor = new TalonFX(shooterConstants.topMotorId(), shooterConstants.CANbus());
        this.bottomMotor = new TalonFX(shooterConstants.bottomMotorId(), shooterConstants.CANbus());

        this.topMotorSim = new CTREPhoenix6TalonFXSim(
                topMotor,
                1,
                new DCMotorSim(
                        DCMotor.getFalcon500(1),
                        1,
                        0.0005
                )
        );

        this.bottomMotorSim = new CTREPhoenix6TalonFXSim(
                bottomMotor,
                1,
                new DCMotorSim(
                        DCMotor.getFalcon500(1),
                        1,
                        0.0005
                )
        );

        this.topDutyCycleOut = new DutyCycleOut(0);
        this.bottomDutyCycleOut = new DutyCycleOut(0);

        this.topBangBangController = new BangBangController();
        this.bottomBangBangController = new BangBangController();

        this._topPosition = topMotor.getPosition();
        this._topVelocity = topMotor.getVelocity();
        this._topTorqueCurrent = topMotor.getTorqueCurrent();
        this._topDutyCycle = topMotor.getDutyCycle();
        this._topDeviceTemp = topMotor.getDeviceTemp();
        this._bottomPosition = bottomMotor.getPosition();
        this._bottomVelocity = bottomMotor.getVelocity();
        this._bottomTorqueCurrent = bottomMotor.getTorqueCurrent();
        this._bottomDutyCycle = bottomMotor.getDutyCycle();
        this._bottomDeviceTemp = bottomMotor.getDeviceTemp();
    }

    @Override
    public void config() {
        final TalonFXConfiguration topTalonFXConfiguration = new TalonFXConfiguration();
        topTalonFXConfiguration.CurrentLimits.StatorCurrentLimit = 50;
        topTalonFXConfiguration.CurrentLimits.StatorCurrentLimitEnable = true;
        topTalonFXConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        topTalonFXConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        topMotor.getConfigurator().apply(topTalonFXConfiguration);

        final TalonFXConfiguration bottomTalonFXConfiguration = new TalonFXConfiguration();
        bottomTalonFXConfiguration.CurrentLimits.StatorCurrentLimit = 50;
        bottomTalonFXConfiguration.CurrentLimits.StatorCurrentLimitEnable = true;
        bottomTalonFXConfiguration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        bottomTalonFXConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        bottomMotor.getConfigurator().apply(bottomTalonFXConfiguration);
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
    public void periodic() {
        topMotor.setControl(topDutyCycleOut.withOutput(
                topBangBangController.calculate(_topVelocity.getValue(), topVelocitySetpoint))
        );
        bottomMotor.setControl(bottomDutyCycleOut.withOutput(
                bottomBangBangController.calculate(_bottomVelocity.getValue(), bottomVelocitySetpoint))
        );

        final double dtSeconds = deltaTime.get();
        topMotorSim.update(dtSeconds);
        bottomMotorSim.update(dtSeconds);
    }

    @Override
    public void setInputs(final double desiredTopVelocity, final double desiredBottomVelocity) {
        this.topVelocitySetpoint = desiredTopVelocity;
        this.bottomVelocitySetpoint = desiredBottomVelocity;
    }
}
