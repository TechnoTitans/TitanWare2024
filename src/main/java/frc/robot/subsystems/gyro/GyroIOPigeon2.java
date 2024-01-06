package frc.robot.subsystems.gyro;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.utils.ctre.Phoenix6Utils;

public class GyroIOPigeon2 implements GyroIO {
    private final Pigeon2 pigeon;

    // Cached StatusSignals
    private final StatusSignal<Boolean> _faultHardware;
    private final StatusSignal<Double> _yaw;
    private final StatusSignal<Double> _pitch;
    private final StatusSignal<Double> _roll;
    private final StatusSignal<Double> _yawVelocity;
    private final StatusSignal<Double> _pitchVelocity;
    private final StatusSignal<Double> _rollVelocity;

    public GyroIOPigeon2(final Pigeon2 pigeon) {
        this.pigeon = pigeon;

        this._faultHardware = pigeon.getFault_Hardware();
        this._yaw = pigeon.getYaw();
        this._pitch = pigeon.getPitch();
        this._roll = pigeon.getRoll();
        this._yawVelocity = pigeon.getAngularVelocityZWorld();
        this._pitchVelocity = pigeon.getAngularVelocityYWorld();
        this._rollVelocity = pigeon.getAngularVelocityXWorld();
    }

    @Override
    public void periodic() {
        BaseStatusSignal.refreshAll(
                _faultHardware,
                _yaw,
                _pitch,
                _roll,
                _yawVelocity,
                _pitchVelocity,
                _rollVelocity
        );
    }

    @Override
    public void updateInputs(final GyroIOInputs inputs) {
        inputs.hasHardwareFault = _faultHardware.getValue();

        inputs.yawPositionDeg = getYaw();
        inputs.pitchPositionDeg = getPitch();
        inputs.rollPositionDeg = getRoll();

        inputs.yawVelocityDegPerSec = _yawVelocity.getValue();
        inputs.pitchVelocityDegPerSec = _pitchVelocity.getValue();
        inputs.rollVelocityDegPerSec = _rollVelocity.getValue();
    }

    @Override
    public void config() {
        //TODO fill in correct mount pose
        final Pigeon2Configuration pigeon2Configuration = new Pigeon2Configuration();
        pigeon2Configuration.MountPose.MountPoseYaw = 0;
        pigeon.getConfigurator().apply(pigeon2Configuration);
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
