package frc.robot.subsystems.gyro;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.sim.Pigeon2SimState;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.constants.HardwareConstants;
import frc.robot.subsystems.drive.Swerve;
import frc.robot.subsystems.drive.SwerveModule;
import frc.robot.utils.control.DeltaTime;

import java.util.Queue;

public class GyroIOSim implements GyroIO {
    public static final double USE_SIMULATED_PITCH = 0;
    public static final double USE_SIMULATED_ROLL = 0;

    private final Pigeon2 pigeon;
    private final Pigeon2SimState pigeonSimState;
    private final SwerveDriveKinematics kinematics;
    private final SwerveModule[] swerveModules;
    private final double[] lastSwerveModulePositionMeters = {0.0, 0.0, 0.0, 0.0};

    private final DeltaTime deltaTime;
    private final LinearFilter filter = LinearFilter.movingAverage(50);
    private Rotation2d rawGyroYaw = Rotation2d.fromDegrees(0);

    // Cached StatusSignals
    private final StatusSignal<Double> _yaw;
    private final StatusSignal<Double> _pitch;
    private final StatusSignal<Double> _roll;
    private final StatusSignal<Double> _yawVelocity;
    private final StatusSignal<Double> _pitchVelocity;
    private final StatusSignal<Double> _rollVelocity;
    private final StatusSignal<Boolean> _faultHardware;

    // StatusSignal queues for high-freq odometry
    private final Queue<Double> timestampQueue;
    private final Queue<Double> yawSignalQueue;

    public GyroIOSim(
            final HardwareConstants.GyroConstants gyroConstants,
            final Swerve.OdometryThreadRunner odometryThreadRunner,
            final SwerveDriveKinematics kinematics,
            final SwerveModule[] swerveModules
    ) {
        this.pigeon = new Pigeon2(gyroConstants.gyroId(), gyroConstants.CANBus());
        this.pigeonSimState = pigeon.getSimState();
        this.kinematics = kinematics;
        this.swerveModules = swerveModules;

        this.deltaTime = new DeltaTime(true);

        this._yaw = pigeon.getYaw();
        this._pitch = pigeon.getPitch();
        this._roll = pigeon.getRoll();
        this._yawVelocity = pigeon.getAngularVelocityZWorld();
        this._pitchVelocity = pigeon.getAngularVelocityXWorld();
        this._rollVelocity = pigeon.getAngularVelocityYWorld();
        this._faultHardware = pigeon.getFault_Hardware();

        this.timestampQueue = odometryThreadRunner.makeTimestampQueue();
        this.yawSignalQueue = odometryThreadRunner.registerSignal(pigeon, _yaw);

        pigeonSimState.setSupplyVoltage(12);
        pigeonSimState.setPitch(USE_SIMULATED_PITCH);
        pigeonSimState.setRoll(USE_SIMULATED_ROLL);
    }

    private void updateGyro(final double dtSeconds) {
        final SwerveModuleState[] moduleStates = new SwerveModuleState[swerveModules.length];
        for (int i = 0; i < moduleStates.length; i++) {
            moduleStates[i] = swerveModules[i].getState();
        }

        rawGyroYaw = rawGyroYaw.plus(Rotation2d.fromRadians(
                kinematics.toChassisSpeeds(moduleStates).omegaRadiansPerSecond * dtSeconds
        ));
        pigeonSimState.setRawYaw(rawGyroYaw.getDegrees());
    }

    @SuppressWarnings("DuplicatedCode")
    @Override
    public void config() {
        //TODO fill in correct mount pose
        final Pigeon2Configuration pigeon2Configuration = new Pigeon2Configuration();
        pigeon2Configuration.MountPose.MountPoseYaw = 0;
        pigeon.getConfigurator().apply(pigeon2Configuration);

        BaseStatusSignal.setUpdateFrequencyForAll(
                100,
                _pitch,
                _pitchVelocity,
                _roll,
                _rollVelocity
        );
        BaseStatusSignal.setUpdateFrequencyForAll(
                4,
                _faultHardware
        );
        ParentDevice.optimizeBusUtilizationForAll(pigeon);
    }

    @Override
    public void periodic() {
        updateGyro(deltaTime.get());
    }

    @SuppressWarnings("DuplicatedCode")
    @Override
    public void updateInputs(final GyroIOInputs inputs) {
        BaseStatusSignal.refreshAll(
                _yaw,
                _pitch,
                _roll,
                _yawVelocity,
                _pitchVelocity,
                _rollVelocity,
                _faultHardware
        );

        inputs.yawPositionDeg = getYaw();
        inputs.pitchPositionDeg = getPitch();
        inputs.rollPositionDeg = getRoll();
        inputs.yawVelocityDegPerSec = _yawVelocity.getValue();
        inputs.pitchVelocityDegPerSec = _pitchVelocity.getValue();
        inputs.rollVelocityDegPerSec = _rollVelocity.getValue();
        inputs.hasHardwareFault = _faultHardware.getValue();

        inputs.odometryTimestampsSec = timestampQueue.stream().mapToDouble(time -> time).toArray();
        timestampQueue.clear();

        inputs.odometryYawPositionsDeg = yawSignalQueue.stream().mapToDouble(yaw -> yaw).toArray();
        yawSignalQueue.clear();
    }

    public double getYaw() {
        // TODO: fairly sure that yaw, pitch, roll velocity don't work in sim, so we can't latency compensate
//        return Phoenix6Utils.latencyCompensateIfSignalIsGood(
//                _yaw,
//                getYawVelocitySignal()
//        );
        return _yaw.getValue();
    }

    public double getPitch() {
        // see previous mention of velocities not working in sim

//        return Phoenix6Utils.latencyCompensateIfSignalIsGood(
//                pigeon.getPitch(),
//                getPitchVelocitySignal()
//        );
        return _pitch.getValue();
    }

    public double getRoll() {
        // see previous mention of velocities not working in sim

//        return Phoenix6Utils.latencyCompensateIfSignalIsGood(
//                pigeon.getRoll(),
//                getRollVelocitySignal()
//        );
        return _roll.getValue();
    }

    @Override
    public void setAngle(final Rotation2d angle) {
        pigeon.setYaw(angle.getDegrees());
    }
}
