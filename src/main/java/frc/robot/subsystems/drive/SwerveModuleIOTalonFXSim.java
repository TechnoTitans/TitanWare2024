package frc.robot.subsystems.drive;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.util.DoubleCircularBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.constants.Constants.Swerve.Modules;
import frc.robot.constants.SimConstants;
import frc.robot.utils.closeables.ToClose;
import frc.robot.utils.control.DeltaTime;
import frc.robot.utils.ctre.Phoenix6Utils;
import frc.robot.utils.sim.SimUtils;
import frc.robot.utils.sim.feedback.SimPhoenix6CANCoder;
import frc.robot.utils.sim.motors.TalonFXSim;

public class SwerveModuleIOTalonFXSim implements SwerveModuleIO {
    private static final double SIM_UPDATE_PERIOD_SEC = 0.005;

    private final TalonFX driveMotor;
    private final TalonFX turnMotor;
    private final TalonFXSim driveSim;
    private final TalonFXSim turnSim;
    private final CANcoder turnEncoder;
    private final double magnetOffset;

    private final TalonFXConfiguration driveTalonFXConfiguration = new TalonFXConfiguration();
    private final TalonFXConfiguration turnTalonFXConfiguration = new TalonFXConfiguration();

    private final VelocityTorqueCurrentFOC velocityTorqueCurrentFOC;
    private final PositionVoltage positionVoltage;

    private final OdometryThreadRunner odometryThreadRunner;

    private final DeltaTime deltaTime;

    // Cached StatusSignals
    private final StatusSignal<Double> _drivePosition;
    private final StatusSignal<Double> _driveVelocity;
    private final StatusSignal<Double> _driveTorqueCurrent;
    private final StatusSignal<Double> _driveStatorCurrent;
    private final StatusSignal<Double> _driveDeviceTemp;
    private final StatusSignal<Double> _turnPosition;
    private final StatusSignal<Double> _turnVelocity;
    private final StatusSignal<Double> _turnTorqueCurrent;
    private final StatusSignal<Double> _turnStatorCurrent;
    private final StatusSignal<Double> _turnDeviceTemp;

    // Odometry StatusSignal update buffers
    private final DoubleCircularBuffer timestampBuffer;
    private final DoubleCircularBuffer drivePositionSignalBuffer;
    private final DoubleCircularBuffer turnPositionSignalBuffer;

    public SwerveModuleIOTalonFXSim(
            final TalonFX driveMotor,
            final TalonFX turnMotor,
            final CANcoder turnEncoder,
            final double magnetOffset,
            final OdometryThreadRunner odometryThreadRunner
    ) {
        this.driveMotor = driveMotor;
        this.turnMotor = turnMotor;

        final DCMotorSim driveDCMotorSim = new DCMotorSim(
                DCMotor.getKrakenX60Foc(1),
                Modules.DRIVER_GEAR_RATIO,
                Modules.DRIVE_WHEEL_MOMENT_OF_INERTIA_KG_M_SQUARED
        );

        this.driveSim = new TalonFXSim(
                driveMotor,
                Modules.DRIVER_GEAR_RATIO,
                driveDCMotorSim::update,
                (motorVoltage) -> driveDCMotorSim.setInputVoltage(
                        SimUtils.addMotorFriction(motorVoltage, Modules.DRIVE_KS_VOLTS)
                ),
                driveDCMotorSim::getAngularPositionRad,
                driveDCMotorSim::getAngularVelocityRadPerSec
        );

        final DCMotorSim turnDCMotorSim = new DCMotorSim(
                DCMotor.getFalcon500Foc(1),
                Modules.TURNER_GEAR_RATIO,
                Modules.TURN_WHEEL_MOMENT_OF_INERTIA_KG_M_SQUARED
        );

        this.turnEncoder = turnEncoder;
        this.magnetOffset = magnetOffset;
        this.turnSim = new TalonFXSim(
                turnMotor,
                Modules.TURNER_GEAR_RATIO,
                turnDCMotorSim::update,
                (motorVoltage) -> turnDCMotorSim.setInputVoltage(
                        SimUtils.addMotorFriction(motorVoltage, Modules.STEER_KS_VOLTS)
                ),
                turnDCMotorSim::getAngularPositionRad,
                turnDCMotorSim::getAngularVelocityRadPerSec
        );
        this.turnSim.attachFeedbackSensor(new SimPhoenix6CANCoder(turnEncoder));

        this.velocityTorqueCurrentFOC = new VelocityTorqueCurrentFOC(0);
        this.positionVoltage = new PositionVoltage(0);
        this.odometryThreadRunner = odometryThreadRunner;
        this.odometryThreadRunner.registerControlRequest(driveMotor, velocityTorqueCurrentFOC, driveMotor::setControl);
        this.odometryThreadRunner.registerControlRequest(turnMotor, positionVoltage, turnMotor::setControl);

        this.deltaTime = new DeltaTime(true);

        this._drivePosition = driveMotor.getPosition();
        this._driveVelocity = driveMotor.getVelocity();
        this._driveTorqueCurrent = driveMotor.getTorqueCurrent();
        this._driveStatorCurrent = driveMotor.getStatorCurrent();
        this._driveDeviceTemp = driveMotor.getDeviceTemp();
        this._turnPosition = turnEncoder.getAbsolutePosition();
        this._turnVelocity = turnEncoder.getVelocity();
        this._turnTorqueCurrent = turnMotor.getTorqueCurrent();
        this._turnStatorCurrent = turnMotor.getStatorCurrent();
        this._turnDeviceTemp = turnMotor.getDeviceTemp();

        this.timestampBuffer = odometryThreadRunner.makeTimestampBuffer();
        this.drivePositionSignalBuffer = odometryThreadRunner.registerSignal(driveMotor, _drivePosition);
        this.turnPositionSignalBuffer = odometryThreadRunner.registerSignal(turnMotor, _turnPosition);

        final Notifier simUpdateNotifier = new Notifier(() -> {
            final double dtSeconds = deltaTime.get();
            driveSim.update(dtSeconds);
            turnSim.update(dtSeconds);
        });
        ToClose.add(simUpdateNotifier);
        simUpdateNotifier.setName(String.format(
                "SimUpdate(%d,%d)",
                driveMotor.getDeviceID(),
                turnMotor.getDeviceID()
        ));
        simUpdateNotifier.startPeriodic(SIM_UPDATE_PERIOD_SEC);
    }

    @SuppressWarnings("DuplicatedCode")
    @Override
    public void config() {
        final CANcoderConfiguration canCoderConfiguration = new CANcoderConfiguration();
        canCoderConfiguration.MagnetSensor.MagnetOffset = -magnetOffset;
        canCoderConfiguration.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
        turnEncoder.getConfigurator().apply(canCoderConfiguration);

        // TODO: I think we need to look at VoltageConfigs and/or CurrentLimitConfigs for limiting the
        //  current we can apply in sim, this is cause we use VelocityVoltage in sim instead of VelocityTorqueCurrentFOC
        //  which means that TorqueCurrent.PeakForwardTorqueCurrent and related won't affect it
        final InvertedValue driveInvertedValue = InvertedValue.CounterClockwise_Positive;
        driveTalonFXConfiguration.Slot0 = new Slot0Configs()
                .withKP(50)
                .withKS(4.796)
                .withKA(2.549);
        driveTalonFXConfiguration.TorqueCurrent.PeakForwardTorqueCurrent = Modules.SLIP_CURRENT_A;
        driveTalonFXConfiguration.TorqueCurrent.PeakReverseTorqueCurrent = -Modules.SLIP_CURRENT_A;
        driveTalonFXConfiguration.CurrentLimits.StatorCurrentLimit = Modules.SLIP_CURRENT_A;
        driveTalonFXConfiguration.CurrentLimits.StatorCurrentLimitEnable = true;
        driveTalonFXConfiguration.ClosedLoopRamps.TorqueClosedLoopRampPeriod = 0.2;
        driveTalonFXConfiguration.Feedback.SensorToMechanismRatio = Modules.DRIVER_GEAR_RATIO;
        driveTalonFXConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        driveTalonFXConfiguration.MotorOutput.Inverted = driveInvertedValue;
        driveMotor.getConfigurator().apply(driveTalonFXConfiguration);

        final InvertedValue turnInvertedValue = InvertedValue.Clockwise_Positive;
        turnTalonFXConfiguration.Slot0 = new Slot0Configs()
                .withKP(30)
                .withKS(0.5);
        turnTalonFXConfiguration.TorqueCurrent.PeakForwardTorqueCurrent = 40;
        turnTalonFXConfiguration.TorqueCurrent.PeakReverseTorqueCurrent = -40;
        turnTalonFXConfiguration.Feedback.FeedbackRemoteSensorID = turnEncoder.getDeviceID();
        turnTalonFXConfiguration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        turnTalonFXConfiguration.Feedback.RotorToSensorRatio = Modules.TURNER_GEAR_RATIO;
        turnTalonFXConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        turnTalonFXConfiguration.MotorOutput.Inverted = turnInvertedValue;
        turnMotor.getConfigurator().apply(turnTalonFXConfiguration);

        velocityTorqueCurrentFOC.UpdateFreqHz = 0;
        positionVoltage.UpdateFreqHz = 0;

        // TODO: this fix for CANCoder initialization in sim doesn't seem to work all the time...investigate!
//      SimUtils.initializeCTRECANCoderSim(turnEncoder);
        SimUtils.setCTRETalonFXSimStateMotorInverted(driveMotor, driveInvertedValue);
        SimUtils.setCTRETalonFXSimStateMotorInverted(turnMotor, turnInvertedValue);
    }

    @SuppressWarnings("DuplicatedCode")
    @Override
    public void updateInputs(final SwerveModuleIOInputs inputs) {
        BaseStatusSignal.refreshAll(
                _drivePosition,
                _driveVelocity,
                _driveTorqueCurrent,
                _driveStatorCurrent,
                _driveDeviceTemp,
                _turnPosition,
                _turnVelocity,
                _turnTorqueCurrent,
                _turnStatorCurrent,
                _turnDeviceTemp
        );

        inputs.drivePositionRots = getDrivePosition();
        inputs.driveVelocityRotsPerSec = _driveVelocity.getValue();
        inputs.driveTorqueCurrentAmps = _driveTorqueCurrent.getValue();
        inputs.driveStatorCurrentAmps = _driveStatorCurrent.getValue();
        inputs.driveTempCelsius = _driveDeviceTemp.getValue();

        inputs.turnAbsolutePositionRots = getRawAngle();
        inputs.turnVelocityRotsPerSec = _turnVelocity.getValue();
        inputs.turnTorqueCurrentAmps = _turnTorqueCurrent.getValue();
        inputs.turnStatorCurrentAmps = _turnStatorCurrent.getValue();
        inputs.turnTempCelsius = _turnDeviceTemp.getValue();

        inputs.odometryTimestampsSec = OdometryThreadRunner.writeBufferToArray(timestampBuffer);
        timestampBuffer.clear();

        inputs.odometryDrivePositionsRots = OdometryThreadRunner.writeBufferToArray(drivePositionSignalBuffer);
        drivePositionSignalBuffer.clear();

        inputs.odometryTurnPositionRots = OdometryThreadRunner.writeBufferToArray(turnPositionSignalBuffer);
        turnPositionSignalBuffer.clear();
    }

    /**
     * Get the measured mechanism (wheel) angle of the {@link SwerveModuleIOTalonFXSim}, in raw units (rotations)
     * @return the measured wheel (turner) angle, in rotations
     */
    private double getRawAngle() {
        return Phoenix6Utils.latencyCompensateIfSignalIsGood(_turnPosition, _turnVelocity);
    }

    public double getDrivePosition() {
        return Phoenix6Utils.latencyCompensateIfSignalIsGood(_drivePosition, _driveVelocity);
    }

    @Override
    public void setInputs(final double desiredDriverVelocity, final double desiredTurnerRotations) {
        odometryThreadRunner.updateControlRequest(driveMotor, velocityTorqueCurrentFOC);
        driveMotor.setControl(velocityTorqueCurrentFOC
                .withVelocity(desiredDriverVelocity)
                .withOverrideCoastDurNeutral(true)
        );
        turnMotor.setControl(positionVoltage.withPosition(desiredTurnerRotations));
    }

    @Override
    public void setNeutralMode(final NeutralModeValue neutralMode) {
        if (SimConstants.CTRE.DISABLE_NEUTRAL_MODE_IN_SIM) {
            // just ignore setNeutralMode call if NeutralMode setting in Simulation is turned off
            return;
        }

        final StatusCode refreshCode = driveMotor.getConfigurator().refresh(turnTalonFXConfiguration, 0.2);
        if (!refreshCode.isOK()) {
            // warn if the refresh call failed in sim, which might happen pretty often as
            // there seems to be an issue with calling refresh while disabled in sim
            DriverStation.reportWarning(
                    String.format(
                            "Failed to set NeutralMode on TalonFX %s (%s)",
                            driveMotor.getDeviceID(),
                            driveMotor.getNetwork()
                    ), false
            );
            return;
        }

        turnTalonFXConfiguration.MotorOutput.NeutralMode = neutralMode;
        driveMotor.getConfigurator().apply(turnTalonFXConfiguration);
    }
}
