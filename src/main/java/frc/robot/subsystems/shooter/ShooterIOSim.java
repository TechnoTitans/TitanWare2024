package frc.robot.subsystems.shooter;

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
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.constants.HardwareConstants;
import frc.robot.utils.control.DeltaTime;
import frc.robot.utils.sim.motors.CTREPhoenix6TalonFXSim;

// TODO: @max pls fix from real impl, and test in sim
public class ShooterIOSim implements ShooterIO {
    private final DeltaTime deltaTime;

    private final TalonFX topMotor;
    private final TalonFX bottomMotor;

    private final CTREPhoenix6TalonFXSim topMotorSim;
    private final CTREPhoenix6TalonFXSim bottomMotorSim;

    private final VelocityVoltage velocityVoltage;
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

        this.velocityVoltage = new VelocityVoltage(0);
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

    @Override
    public void config() {
        final TalonFXConfiguration topTalonFXConfiguration = new TalonFXConfiguration();
        topTalonFXConfiguration.Slot0 = new Slot0Configs()
                .withKS(0.094)
                .withKV(0.121)
                .withKA(0.01)
                .withKP(1); // TODO: tune Kp
        topTalonFXConfiguration.CurrentLimits.StatorCurrentLimit = 80;
        topTalonFXConfiguration.CurrentLimits.StatorCurrentLimitEnable = true;
        topTalonFXConfiguration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        topTalonFXConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        topMotor.getConfigurator().apply(topTalonFXConfiguration);

        final TalonFXConfiguration bottomTalonFXConfiguration = new TalonFXConfiguration();
        bottomTalonFXConfiguration.Slot0 = new Slot0Configs()
                .withKS(0.11)
                .withKV(0.122)
                .withKA(0.011)
                .withKP(1); // TODO: tune Kp
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
    public void periodic() {
        final double dtSeconds = deltaTime.get();
        topMotorSim.update(dtSeconds);
        bottomMotorSim.update(dtSeconds);
    }

    @Override
    public void setInputs(final double desiredTopVelocity, final double desiredBottomVelocity) {
        topMotor.setControl(velocityVoltage.withVelocity(desiredTopVelocity));
        bottomMotor.setControl(velocityVoltage.withVelocity(desiredBottomVelocity));
    }

    @Override
    public void setCharacterizationVolts(double topVolts, double bottomVolts) {
        topMotor.setControl(voltageOut.withOutput(topVolts));
        bottomMotor.setControl(voltageOut.withOutput(bottomVolts));
    }
}
