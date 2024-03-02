package frc.robot.subsystems.superstructure;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.superstructure.arm.Arm;
import frc.robot.subsystems.superstructure.shooter.Shooter;

public class Superstructure {
    private final Arm arm;
    private final Shooter shooter;

    public final Trigger atGoalTrigger;

    public record SuperstructureSetpoint(
            double armPivotPositionRots,
            double ampVelocityRotsPerSec,
            double leftVelocityRotsPerSec,
            double rightVelocityRotsPerSec
    ) {}

    public record VoltageSetpoint(
            double armPivotVolts,
            double ampVolts,
            double leftVolts,
            double rightVolts
    ) {}

    public enum Goal {
        IDLE(Arm.Goal.STOW, Shooter.Goal.IDLE),
        SUBWOOFER(Arm.Goal.SUBWOOFER, Shooter.Goal.SUBWOOFER);

        private final Arm.Goal armGoal;
        private final Shooter.Goal shooterGoal;

        Goal(final Arm.Goal armGoal, final Shooter.Goal shooterGoal) {
            this.armGoal = armGoal;
            this.shooterGoal = shooterGoal;
        }
    }

    public Superstructure(final Arm arm, final Shooter shooter) {
        this.arm = arm;
        this.shooter = shooter;
        this.atGoalTrigger = arm.atPivotPositionTrigger.and(shooter.atVelocityTrigger);
    }

    public Command toGoal(final Goal goal) {
        return Commands.parallel(
                arm.toGoal(goal.armGoal),
                shooter.toGoal(goal.shooterGoal)
        );
    }

    public Command toSetpoint(final SuperstructureSetpoint superstructureSetpoint) {
        return Commands.parallel(
                arm.toPivotPositionCommand(superstructureSetpoint.armPivotPositionRots),
                shooter.toVelocityCommand(
                        superstructureSetpoint.ampVelocityRotsPerSec,
                        superstructureSetpoint.leftVelocityRotsPerSec,
                        superstructureSetpoint.rightVelocityRotsPerSec
                )
        );
    }

    public Command toVoltageSetpoint(final VoltageSetpoint voltageSetpoint) {
        return Commands.parallel(
                arm.toPivotVoltageCommand(voltageSetpoint.armPivotVolts),
                shooter.toVoltageCommand(
                        voltageSetpoint.ampVolts,
                        voltageSetpoint.leftVolts,
                        voltageSetpoint.rightVolts
                )
        );
    }

    @SuppressWarnings("unused")
    public Command runVoltageCharacterization() {
        return Commands.parallel(
                arm.torqueCurrentSysIdCommand(),
                shooter.torqueCurrentSysIdCommand()
        );
    }

    @SuppressWarnings("unused")
    public Command runTorqueCurrentCharacterization() {
        return Commands.parallel(
                arm.torqueCurrentSysIdCommand(),
                shooter.torqueCurrentSysIdCommand()
        );
    }
}
