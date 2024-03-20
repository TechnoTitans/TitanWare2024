package frc.robot.subsystems.superstructure;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.superstructure.arm.Arm;
import frc.robot.subsystems.superstructure.shooter.Shooter;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class Superstructure {
    private final Arm arm;
    private final Shooter shooter;

    public final Trigger atSetpoint;

    public enum Goal {
        IDLE(Arm.Goal.STOW, Shooter.Goal.IDLE),
        SUBWOOFER(Arm.Goal.SUBWOOFER, Shooter.Goal.SUBWOOFER),
        AMP(Arm.Goal.AMP, Shooter.Goal.AMP),
        EJECT(Arm.Goal.STOW, Shooter.Goal.EJECT),
        BACK_FEED(Arm.Goal.STOW, Shooter.Goal.BACK_FEED);

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
        this.atSetpoint = arm.atPivotSetpoint.and(shooter.atVelocitySetpoint);
    }

    public Command home() {
        return Commands.sequence(
                arm.homePivotCommand()
        );
    }

    public Command toGoal(final Goal goal) {
        return Commands.parallel(
                arm.toGoal(goal.armGoal),
                shooter.toGoal(goal.shooterGoal)
        );
    }

    public Command runState(final Supplier<ShotParameters.Parameters> parametersSupplier) {
        return runState(
                () -> parametersSupplier.get().armPivotAngle(),
                () -> parametersSupplier.get().ampVelocityRotsPerSec(),
                () -> parametersSupplier.get().leftVelocityRotsPerSec(),
                () -> parametersSupplier.get().rightVelocityRotsPerSec()
        );
    }

    public Command runState(
            final Supplier<Rotation2d> armPivotPosition,
            final DoubleSupplier ampVelocityRotsPerSec,
            final DoubleSupplier leftVelocityRotsPerSec,
            final DoubleSupplier rightVelocityRotsPerSec
    ) {
        return Commands.parallel(
                arm.toPivotPositionCommand(() -> armPivotPosition.get().getRotations()),
                shooter.toVelocityCommand(ampVelocityRotsPerSec, leftVelocityRotsPerSec, rightVelocityRotsPerSec)
        );
    }

    public Command runVoltageCommand(
            final double armPivotVolts,
            final double ampVolts,
            final double leftVolts,
            final double rightVolts
    ) {
        return Commands.parallel(
                arm.runPivotVoltageCommand(armPivotVolts),
                shooter.runVoltageCommand(
                        ampVolts,
                        leftVolts,
                        rightVolts
                )
        );
    }

    @SuppressWarnings("unused")
    public Command runVoltageCharacterization() {
        return Commands.parallel(
                arm.voltageSysIdCommand(),
                shooter.voltageSysIdCommand()
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
