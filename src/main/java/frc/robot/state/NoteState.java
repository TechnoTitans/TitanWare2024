package frc.robot.state;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.Constants;
import frc.robot.subsystems.intake.Intake;
import frc.robot.utils.subsystems.VirtualSubsystem;
import org.littletonrobotics.junction.Logger;

import java.util.concurrent.ThreadLocalRandom;
import java.util.function.BooleanSupplier;

public class NoteState extends VirtualSubsystem {
    public static final String LogKey = "NoteState";
    private final Intake intake;

    public enum State {
        NONE,
        INTAKING,
        STORING_FORWARD,
        STORING_BACKWARD,
        STORED,
        FEEDING
    }

    private final ThreadLocalRandom random = ThreadLocalRandom.current();

    private State state = State.NONE;
    public final Trigger isNone = isStateTrigger(State.NONE);
    public final Trigger isIntaking = isStateTrigger(State.INTAKING);
    public final Trigger isStoringForward = isStateTrigger(State.STORING_FORWARD);
    public final Trigger isStoringBackward = isStateTrigger(State.STORING_BACKWARD);
    public final Trigger isStored = isStateTrigger(State.STORED);
    public final Trigger isFeeding = isStateTrigger(State.FEEDING);

    public final Trigger isStoring = isStoringForward.or(isStoringBackward);
    public final Trigger hasNote = isStoring.or(isStored).or(isFeeding);

    public final BooleanSupplier shouldStoreNotes;

    public NoteState(final Constants.RobotMode mode, final Intake intake) {
        this.intake = intake;
        this.shouldStoreNotes = intake.shouldStoreNotes;

        configureStateTriggers();
        if (mode != Constants.RobotMode.REAL) {
            configureSimTriggers();
        }
    }

    @Override
    public void periodic() {
        Logger.recordOutput(LogKey + "/State", state.toString());
        Logger.recordOutput(LogKey + "/HasNote", hasNote.getAsBoolean());
        Logger.recordOutput(LogKey + "/IsStoring", isStored.getAsBoolean());
    }

    public Trigger isStateTrigger(final State state) {
        return new Trigger(() -> this.state == state);
    }

    public Command setState(final State state) {
        return Commands.runOnce(() -> this.state = state);
    }

    public State getState() {
        return state;
    }

    public void configureStateTriggers() {
        intake.intaking.and(hasNote.negate())
                .onTrue(setState(State.INTAKING));
        intake.intaking.negate().and(isIntaking)
                .onTrue(setState(State.NONE));

        intake.feeding.and(isStored).and(intake.shooterBeamBreakBroken)
                .onTrue(setState(State.FEEDING));
        isFeeding.and(intake.shooterBeamBreakBroken.negate())
                .onTrue(setState(State.NONE));

        intake.intaking.and(isStored).and(isStoring.negate()).and(intake.shooterBeamBreakBroken.negate())
                .onTrue(Commands.either(
                        Commands.sequence(
                                setState(State.STORING_FORWARD),
                                intake.storeCommand()
                        ),
                        setState(State.STORED),
                        shouldStoreNotes
                ));
        intake.intaking.and(intake.shooterBeamBreakBroken)
                .onTrue(Commands.either(
                        Commands.sequence(
                                setState(State.STORING_BACKWARD),
                                intake.storeCommand()
                        ),
                        setState(State.STORED),
                        shouldStoreNotes
                ));
        isStoringForward.and(intake.shooterBeamBreakBroken)
                .onTrue(setState(State.STORING_BACKWARD));
        isStoringBackward.and(intake.shooterBeamBreakBroken.negate())
                .onTrue(setState(State.STORED));
    }

    private Command waitRand(final double lowerInclusiveSeconds, final double upperExclusiveSeconds) {
        return Commands.waitSeconds(random.nextDouble(lowerInclusiveSeconds, upperExclusiveSeconds));
    }

    private Command setIntakeSensorState(final boolean shooterBeamBroken) {
        return Commands.runOnce(() -> intake.setBeamBreakSensorState(shooterBeamBroken));
    }

    public void configureSimTriggers() {
        intake.intaking.and(hasNote.negate()).and(intake.shooterBeamBreakBroken.negate())
                .whileTrue(Commands.sequence(
                        waitRand(0.1, 2),
                        Commands.waitSeconds(0.15),
                        setIntakeSensorState(true)
                ));
        intake.intaking.and(hasNote).and(intake.shooterBeamBreakBroken.negate())
                .whileTrue(Commands.sequence(
                        Commands.waitSeconds(0.15),
                        setIntakeSensorState(true)
                ));
        isStoringForward.and(intake.shooterBeamBreakBroken.negate())
                .whileTrue(Commands.sequence(
                        waitRand(0.05, 0.1),
                        setIntakeSensorState(true)
                ));
        isStoringBackward.and(intake.shooterBeamBreakBroken)
                .whileTrue(Commands.sequence(
                        waitRand(0.05, 0.1),
                        setIntakeSensorState(false)
                ));

        intake.feeding.and(isStored)
                .onTrue(Commands.sequence(
                        waitRand(0.01, 0.1),
                        setIntakeSensorState(true),
                        waitRand(0.02, 0.1),
                        setIntakeSensorState(false)
                ));
    }
}
