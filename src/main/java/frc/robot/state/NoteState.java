package frc.robot.state;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.superstructure.Superstructure;

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

    private State state;
    private final Trigger isNone = inStateTrigger(State.NONE);
    private final Trigger isInvalid = inStateTrigger(State.INVALID);
    private final Trigger isIntaking = inStateTrigger(State.INTAKING);
    private final Trigger isStoring = inStateTrigger(State.STORING);
    private final Trigger isStored = inStateTrigger(State.STORED);
    private final Trigger isFeeding = inStateTrigger(State.FEEDING);

    public NoteState(final Intake intake, final Superstructure superstructure) {
        this.intake = intake;
        this.superstructure = superstructure;
    }

    public Trigger inStateTrigger(final State state) {
        return new Trigger(() -> this.state == state);
    }

    public Command setState(final State state) {
        return Commands.runOnce(() -> this.state = state);
    }

    public void configureStateBindings() {
        intake.intaking.and(isNone)
                .onTrue(setState(State.INTAKING));

        intake.feeding.and(isStored).and(intake.shooterBeamBreakBroken)
                .onTrue(setState(State.FEEDING));
        isFeeding.and(intake.shooterBeamBreakBroken.negate())
                .onTrue(setState(State.NONE));

        isIntaking.and(isNone.negate())
                .onTrue(setState(State.INVALID));

        isIntaking.and(isNone).and(intake.shooterBeamBreakBroken)
                .onTrue(Commands.sequence(
                        setState(State.STORING),
                        intake.storeCommand(),
                        setState(State.STORED)
                ));
    }
}
