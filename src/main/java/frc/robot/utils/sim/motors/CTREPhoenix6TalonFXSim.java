package frc.robot.utils.sim.motors;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.constants.SimConstants;
import frc.robot.utils.sim.feedback.SimFeedbackSensor;

import java.util.List;

public class CTREPhoenix6TalonFXSim implements SimMotorController {
    private final List<TalonFXSimState> simStates;
    private final DCMotorSim dcMotorSim;
    private final double gearRatio;

    private final boolean isSingularTalonFX;

    private boolean hasRemoteSensor = false;
    private SimFeedbackSensor feedbackSensor;

    private CTREPhoenix6TalonFXSim(
            final List<TalonFX> talonFXControllers,
            final List<TalonFXSimState> simStates,
            final DCMotorSim dcMotorSim,
            final double gearRatio
    ) {
        if (talonFXControllers.isEmpty() || simStates.isEmpty()) {
            throw new IllegalArgumentException("TalonFX must not be empty! TalonFXSimStates must not be empty!");
        }

        this.simStates = simStates;
        this.dcMotorSim = dcMotorSim;
        this.gearRatio = gearRatio;

        this.isSingularTalonFX = talonFXControllers.size() == 1 && simStates.size() == 1;
    }

    public CTREPhoenix6TalonFXSim(
            final List<TalonFX> talonFXControllers,
            final double gearRatio,
            final DCMotorSim motorSim
    ) {
        this(talonFXControllers, talonFXControllers.stream().map(TalonFX::getSimState).toList(), motorSim, gearRatio);
    }

    public CTREPhoenix6TalonFXSim(final TalonFX talonSRX, final double gearRatio, final DCMotorSim motorSim) {
        this(List.of(talonSRX), gearRatio, motorSim);
    }

    @Override
    public void attachFeedbackSensor(final SimFeedbackSensor feedbackSensor) {
        if (hasRemoteSensor) {
            throw new RuntimeException("Attempt to attach SimFeedbackSensor when one is already attached!");
        }

        this.hasRemoteSensor = true;
        this.feedbackSensor = feedbackSensor;
    }

    @Override
    public void update(final double dt) {
        final double motorVoltage = getMotorVoltage();
        dcMotorSim.setInputVoltage(motorVoltage);
        dcMotorSim.update(dt);

        final double mechanismAngularPositionRots = getAngularPositionRots();
        final double mechanismAngularVelocityRotsPerSec = getAngularVelocityRotsPerSec();

        for (final TalonFXSimState simState : simStates) {
            simState.setRawRotorPosition(gearRatio * mechanismAngularPositionRots);
            simState.setRotorVelocity(gearRatio * mechanismAngularVelocityRotsPerSec);
            simState.setSupplyVoltage(
                    12 - (simState.getSupplyCurrent() * SimConstants.FALCON_MOTOR_RESISTANCE)
            );
        }

        if (hasRemoteSensor) {
            feedbackSensor.setRawPosition(mechanismAngularPositionRots);
            feedbackSensor.setVelocity(mechanismAngularVelocityRotsPerSec);
        }
    }

    @Override
    public void rawUpdate(final double mechanismPositionRots, final double mechanismVelocityRotsPerSec) {
        for (final TalonFXSimState simState : simStates) {
            simState.setRawRotorPosition(gearRatio * mechanismPositionRots);
            simState.setRotorVelocity(gearRatio * mechanismVelocityRotsPerSec);
        }

        if (hasRemoteSensor) {
            feedbackSensor.setRawPosition(mechanismPositionRots);
            feedbackSensor.setVelocity(mechanismVelocityRotsPerSec);
        }
    }

    @Override
    public double getAngularPositionRots() {
        return dcMotorSim.getAngularPositionRotations();
    }

    @Override
    public double getAngularVelocityRotsPerSec() {
        return Units.radiansToRotations(dcMotorSim.getAngularVelocityRadPerSec());
    }

    @Override
    public double getMotorVoltage() {
        return isSingularTalonFX
                ? simStates.get(0).getMotorVoltage()
                : simStates.stream().mapToDouble(TalonFXSimState::getMotorVoltage).average().orElseThrow();
    }

    @Override
    public double getMotorCurrent() {
        return isSingularTalonFX
                ? simStates.get(0).getTorqueCurrent()
                : simStates.stream().mapToDouble(TalonFXSimState::getTorqueCurrent).average().orElseThrow();
    }
}
