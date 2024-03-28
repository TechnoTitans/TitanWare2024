package frc.robot.subsystems.climb;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.units.Current;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Time;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.constants.Constants;
import frc.robot.constants.HardwareConstants;
import frc.robot.utils.logging.LogUtils;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.*;

public class Climb extends SubsystemBase {
    protected static final String LogKey = "Climb";

    private HardwareConstants.ClimbConstants climbConstants;

    private final ClimbIO climbIO;
    private final ClimbIOInputsAutoLogged inputs;

    private final PositionSetpoint setpoint;
    private final Trigger atUpperLimit;
    private final Trigger atLowerLimit;

    private static class PositionSetpoint {
        public double rightArmPositionRots;
        public double leftArmPositionRots;
    }

    private final SysIdRoutine torqueCurrentSysIdRoutine;

    public Climb(
            final Constants.RobotMode robotMode,
            final HardwareConstants.ClimbConstants climbConstants
    ) {
        this.climbConstants = climbConstants;

        this.climbIO = switch (robotMode) {
            case REAL -> new ClimbIOReal(climbConstants);
            case SIM -> new ClimbIOSim(climbConstants);
            case REPLAY -> new ClimbIO() {
            };
        };

        this.inputs = new ClimbIOInputsAutoLogged();
        this.setpoint = new PositionSetpoint();
        this.atUpperLimit = new Trigger(this::atArmUpperLimit);
        this.atLowerLimit = new Trigger(this::atArmLowerLimit);

        this.torqueCurrentSysIdRoutine = makeTorqueCurrentSysIdRoutine(
                Amps.of(2).per(Second),
                Amps.of(10),
                Seconds.of(10)
        );

        this.climbIO.config();
        this.climbIO.initialize();
    }

    @Override
    public void periodic() {
        final double climbIOPeriodicStart = Logger.getRealTimestamp();
        climbIO.updateInputs(inputs);
        Logger.processInputs(LogKey, inputs);

        Logger.recordOutput(
                LogKey + "/PeriodicIOPeriodMs",
                LogUtils.microsecondsToMilliseconds(Logger.getRealTimestamp() - climbIOPeriodicStart)
        );

        Logger.recordOutput(LogKey + "/Setpoint/RightArmPositionRots", setpoint.rightArmPositionRots);
        Logger.recordOutput(LogKey + "/Setpoint/LeftArmPositionRots", setpoint.leftArmPositionRots);
    }

    public Command climbCommand() {
        return runPositionCommand(5, 5);
    }

    private boolean atArmUpperLimit() {
        return inputs.rightPositionRots >= climbConstants.softUpperLimitRots()
                && inputs.leftPositionRots >= climbConstants.softUpperLimitRots();
    }

    private boolean atArmLowerLimit() {
        return inputs.rightPositionRots <= climbConstants.softLowerLimitRots()
                && inputs.leftPositionRots <= climbConstants.softLowerLimitRots();
    }

    public Command runPositionCommand(
            final double rightArmPositionRots,
            final double leftArmPositionRots
    ) {
        return runEnd(() -> {
            setpoint.rightArmPositionRots = rightArmPositionRots;
            setpoint.leftArmPositionRots = leftArmPositionRots;

            climbIO.toPosition(rightArmPositionRots, leftArmPositionRots);
        }, () -> climbIO.toPosition(0, 0));
    }

    private SysIdRoutine makeTorqueCurrentSysIdRoutine(
            final Measure<Velocity<Current>> currentRampRate,
            final Measure<Current> stepCurrent,
            final Measure<Time> timeout
    ) {
        return new SysIdRoutine(
                new SysIdRoutine.Config(
                        // we need to lie to SysId here, because it only takes voltage instead of current
                        Volts.per(Second).of(currentRampRate.baseUnitMagnitude()),
                        Volts.of(stepCurrent.baseUnitMagnitude()),
                        timeout,
                        state -> SignalLogger.writeString("state", state.toString())
                ),
                new SysIdRoutine.Mechanism(
                        voltageMeasure -> climbIO.toTorqueCurrent(
                                // this is really in amps, not volts
                                voltageMeasure.in(Volts),
                                voltageMeasure.in(Volts)
                        ),
                        null,
                        this
                )
        );
    }

    public Command runSysIDRoutineTorqueCurrent() {
        return Commands.sequence(
                torqueCurrentSysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward)
                        .until(atUpperLimit),
                Commands.waitSeconds(2),
                torqueCurrentSysIdRoutine.quasistatic(SysIdRoutine.Direction.kReverse)
                        .until(atLowerLimit),
                Commands.waitSeconds(2),
                torqueCurrentSysIdRoutine.dynamic(SysIdRoutine.Direction.kForward)
                        .until(atUpperLimit),
                Commands.waitSeconds(2),
                torqueCurrentSysIdRoutine.dynamic(SysIdRoutine.Direction.kReverse)
                        .until(atLowerLimit),
                Commands.waitSeconds(2),
                Commands.runOnce(SignalLogger::stop)
        );
    }
}
