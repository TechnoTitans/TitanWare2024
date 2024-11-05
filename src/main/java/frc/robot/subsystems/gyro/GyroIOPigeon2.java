package frc.robot.subsystems.gyro;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.DoubleCircularBuffer;
import frc.robot.constants.HardwareConstants;
import frc.robot.subsystems.drive.OdometryThreadRunner;
import frc.robot.utils.ctre.Phoenix6Utils;

public class GyroIOPigeon2 implements GyroIO {
    private final Pigeon2 pigeon;

    // Cached StatusSignals
    private final StatusSignal<Double> _yaw;
    private final StatusSignal<Double> _pitch;
    private final StatusSignal<Double> _roll;
    private final StatusSignal<Double> _yawVelocity;
    private final StatusSignal<Double> _pitchVelocity;
    private final StatusSignal<Double> _rollVelocity;
    private final StatusSignal<Boolean> _faultHardware;

    // StatusSignal buffers for high-freq odometry
    private final DoubleCircularBuffer timestampBuffer;
    private final DoubleCircularBuffer yawSignalBuffer;

    public GyroIOPigeon2(
            final HardwareConstants.GyroConstants gyroConstants,
            final OdometryThreadRunner odometryThreadRunner
    ) {
        this.pigeon = new Pigeon2(gyroConstants.gyroId(), gyroConstants.CANBus());

        this._yaw = pigeon.getYaw();
        this._pitch = pigeon.getPitch();
        this._roll = pigeon.getRoll();
        this._yawVelocity = pigeon.getAngularVelocityZWorld();
        this._pitchVelocity = pigeon.getAngularVelocityYWorld();
        this._rollVelocity = pigeon.getAngularVelocityXWorld();
        this._faultHardware = pigeon.getFault_Hardware();

        this.timestampBuffer = odometryThreadRunner.makeTimestampBuffer();
        this.yawSignalBuffer = odometryThreadRunner.registerSignal(pigeon, _yaw);
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

        inputs.odometryTimestampsSec = OdometryThreadRunner.writeBufferToArray(timestampBuffer);
        timestampBuffer.clear();

        inputs.odometryYawPositionsDeg = OdometryThreadRunner.writeBufferToArray(yawSignalBuffer);
        yawSignalBuffer.clear();
    }

    // TODO: duplicated code warnings here aren't exactly amazing, but I can't really think of a way to extract
    //  them correctly while still keeping with how AdvKit works
    @SuppressWarnings("DuplicatedCode")
    @Override
    public void config() {
        //TODO fill in correct mount pose
        final Pigeon2Configuration pigeon2Configuration = new Pigeon2Configuration();
        pigeon2Configuration.MountPose.MountPoseRoll = 0;
        pigeon2Configuration.MountPose.MountPosePitch = 0;
        pigeon2Configuration.MountPose.MountPoseYaw = -90;
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

    public double getYaw() {
        return Phoenix6Utils.latencyCompensateIfSignalIsGood(_yaw, _yawVelocity);
    }

    public double getPitch() {
        return Phoenix6Utils.latencyCompensateIfSignalIsGood(_pitch, _pitchVelocity);
    }

    public double getRoll() {
        return Phoenix6Utils.latencyCompensateIfSignalIsGood(_roll, _rollVelocity);
    }

    @Override
    public void setAngle(final Rotation2d angle) {
        pigeon.setYaw(angle.getDegrees());
    }
}
