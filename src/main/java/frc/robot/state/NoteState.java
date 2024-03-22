package frc.robot.state;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.superstructure.Superstructure;

// TODO: get this working before district champs
public class NoteState {
    private final Intake intake;
    private final Superstructure superstructure;

    public enum State {
        NONE,
        INVALID,
        INTAKING,
        STORING,
        STORED,
        FEEDING
    }

    private State state = State.NONE;
    public final Trigger isNone = isStateTrigger(State.NONE);
    public final Trigger isInvalid = isStateTrigger(State.INVALID);
    public final Trigger isIntaking = isStateTrigger(State.INTAKING);
    public final Trigger isStoring = isStateTrigger(State.STORING);
    public final Trigger isStored = isStateTrigger(State.STORED);
    public final Trigger isFeeding = isStateTrigger(State.FEEDING);

    public final Trigger hasNote = isStoring.or(isStored).or(isFeeding);

    public NoteState(final Intake intake, final Superstructure superstructure) {
        this.intake = intake;
        this.superstructure = superstructure;

//        configureStateBindings();
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

    public void configureStateBindings() {
        intake.intaking.and(hasNote.negate())
                .onTrue(setState(State.INTAKING));
        intake.intaking.and(hasNote)
                .onTrue(setState(State.INVALID));

        intake.feeding.and(isStored).and(intake.shooterBeamBreakBroken)
                .onTrue(setState(State.FEEDING));
        isFeeding.and(intake.shooterBeamBreakBroken.negate())
                .onTrue(setState(State.NONE));

        isIntaking.and(isInvalid.negate()).and(intake.shooterBeamBreakBroken)
                .onTrue(Commands.sequence(
                        setState(State.STORING),
                        intake.storeCommand(),
                        setState(State.STORED)
                ));

//        intake.shooterBeamBreakBroken.negate().and(isFeeding.negate()).and(isStoring.negate())
//                .onTrue(setState(State.NONE));
    }
}
