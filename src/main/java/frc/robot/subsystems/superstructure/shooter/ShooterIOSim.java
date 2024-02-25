package frc.robot.subsystems.superstructure.shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.constants.HardwareConstants;
import frc.robot.utils.closeables.ToClose;
import frc.robot.utils.control.DeltaTime;
import frc.robot.utils.sim.SimUtils;
import frc.robot.utils.sim.motors.CTREPhoenix6TalonFXSim;

public class ShooterIOSim implements ShooterIO {
    private static final double SIM_UPDATE_PERIOD_SEC = 0.005;

    private final DeltaTime deltaTime;
    private final HardwareConstants.ShooterConstants shooterConstants;

    private final TalonFX leftFlywheelMotor;
    private final TalonFX rightFlywheelMotor;

    private final CTREPhoenix6TalonFXSim leftMotorSim;
    private final CTREPhoenix6TalonFXSim rightMotorSim;

    private final VelocityVoltage velocityVoltage;
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

    public ShooterIOSim(final HardwareConstants.ShooterConstants shooterConstants) {
        this.deltaTime = new DeltaTime(true);
        this.shooterConstants = shooterConstants;

        this.leftFlywheelMotor = new TalonFX(shooterConstants.leftFlywheelMotorId(), shooterConstants.CANBus());
        this.rightFlywheelMotor = new TalonFX(shooterConstants.rightFlywheelMotorId(), shooterConstants.CANBus());

        final FlywheelSim leftFlywheelSim = new FlywheelSim(
                LinearSystemId.identifyVelocitySystem(
                        0.121 / (2 * Math.PI),
                        0.01 / (2 * Math.PI)
                ),
                DCMotor.getFalcon500(1),
                shooterConstants.leftFlywheelGearing()
        );

        this.leftMotorSim = new CTREPhoenix6TalonFXSim(
                leftFlywheelMotor,
                shooterConstants.leftFlywheelGearing(),
                leftFlywheelSim::update,
                leftFlywheelSim::setInputVoltage,
                () -> 0d,
                leftFlywheelSim::getAngularVelocityRadPerSec
        );

        final FlywheelSim rightFlywheelSim = new FlywheelSim(
                LinearSystemId.identifyVelocitySystem(
                        0.122 / (2 * Math.PI),
                        0.011 / (2 * Math.PI)
                ),
                DCMotor.getFalcon500(1),
                shooterConstants.rightFlywheelGearing()
        );

        this.rightMotorSim = new CTREPhoenix6TalonFXSim(
                rightFlywheelMotor,
                shooterConstants.rightFlywheelGearing(),
                rightFlywheelSim::update,
                rightFlywheelSim::setInputVoltage,
                () -> 0d,
                rightFlywheelSim::getAngularVelocityRadPerSec
        );

        this.velocityVoltage = new VelocityVoltage(0);
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

        final Notifier simUpdateNotifier = new Notifier(() -> {
            final double dt = deltaTime.get();
            leftMotorSim.update(dt);
            rightMotorSim.update(dt);
        });
        ToClose.add(simUpdateNotifier);
        simUpdateNotifier.setName(String.format(
                "SimUpdate(%d,%d)",
                leftFlywheelMotor.getDeviceID(),
                rightFlywheelMotor.getDeviceID()
        ));
        simUpdateNotifier.startPeriodic(SIM_UPDATE_PERIOD_SEC);
    }

    @SuppressWarnings("DuplicatedCode")
    @Override
    public void config() {
        final TalonFXConfiguration leftTalonFXConfiguration = new TalonFXConfiguration();
        final InvertedValue leftTalonFXInverted = InvertedValue.CounterClockwise_Positive;
        leftTalonFXConfiguration.Slot0 = new Slot0Configs()
                .withKS(0.094)
                .withKV(0.121)
                .withKA(0.01)
                .withKP(0.177);
        leftTalonFXConfiguration.CurrentLimits.StatorCurrentLimit = 80;
        leftTalonFXConfiguration.CurrentLimits.StatorCurrentLimitEnable = true;
        leftTalonFXConfiguration.Feedback.SensorToMechanismRatio = shooterConstants.leftFlywheelGearing();
        leftTalonFXConfiguration.MotorOutput.Inverted = leftTalonFXInverted;
        leftTalonFXConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        leftFlywheelMotor.getConfigurator().apply(leftTalonFXConfiguration);

        final TalonFXConfiguration rightTalonFXConfiguration = new TalonFXConfiguration();
        final InvertedValue rightTalonFXInverted = InvertedValue.Clockwise_Positive;
        rightTalonFXConfiguration.Slot0 = new Slot0Configs()
                .withKS(0.11)
                .withKV(0.122)
                .withKA(0.011)
                .withKP(0.17542);
        rightTalonFXConfiguration.CurrentLimits.StatorCurrentLimit = 80;
        rightTalonFXConfiguration.CurrentLimits.StatorCurrentLimitEnable = true;
        rightTalonFXConfiguration.Feedback.SensorToMechanismRatio = shooterConstants.rightFlywheelGearing();
        rightTalonFXConfiguration.MotorOutput.Inverted = rightTalonFXInverted;
        rightTalonFXConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        rightFlywheelMotor.getConfigurator().apply(rightTalonFXConfiguration);

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

        SimUtils.setCTRETalonFXSimStateMotorInverted(leftFlywheelMotor, leftTalonFXInverted);
        SimUtils.setCTRETalonFXSimStateMotorInverted(rightFlywheelMotor, rightTalonFXInverted);
    }

    @SuppressWarnings("DuplicatedCode")
    @Override
    public void updateInputs(final ShooterIOInputs inputs) {
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
    public void toVelocity(
            final double ampVelocity,
            final double leftFlywheelVelocity,
            final double rightFlywheelVelocity
    ) {
        leftFlywheelMotor.setControl(velocityVoltage.withVelocity(leftFlywheelVelocity));
        rightFlywheelMotor.setControl(velocityVoltage.withVelocity(rightFlywheelVelocity));
    }

    @Override
    public void toVoltage(
            final double ampVolts,
            final double leftVolts,
            final double rightVolts
    ) {
        leftFlywheelMotor.setControl(voltageOut.withOutput(leftVolts));
        rightFlywheelMotor.setControl(voltageOut.withOutput(rightVolts));
    }
}
