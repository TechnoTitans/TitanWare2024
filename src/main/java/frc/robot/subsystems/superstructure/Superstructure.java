package frc.robot.subsystems.superstructure;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.superstructure.arm.Arm;
import frc.robot.subsystems.superstructure.shooter.Shooter;
import frc.robot.utils.subsystems.VirtualSubsystem;
import org.littletonrobotics.junction.Logger;

import java.util.Set;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class Superstructure extends VirtualSubsystem {
    public static final String LogKey = "Superstructure";
    private final Arm arm;
    private final Shooter shooter;

    private Goal desiredGoal = Goal.IDLE;
    private Goal currentGoal = desiredGoal;
    public final Trigger atSetpoint;

    public enum Goal {
        NONE(Arm.Goal.NONE, Shooter.Goal.NONE),
        IDLE(Arm.Goal.STOW, Shooter.Goal.IDLE),
        SUBWOOFER(Arm.Goal.SUBWOOFER, Shooter.Goal.SUBWOOFER),
        AMP(Arm.Goal.AMP, Shooter.Goal.AMP),
        TRAP(Arm.Goal.TRAP, Shooter.Goal.TRAP),
        EJECT(Arm.Goal.STOW, Shooter.Goal.EJECT),
        FERRY_CENTERLINE(Arm.Goal.FERRY_CENTERLINE, Shooter.Goal.FERRY_CENTERLINE),
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
        this.atSetpoint = arm.atPivotSetpoint
                .and(shooter.atVelocitySetpoint)
                .and(() -> currentGoal == desiredGoal);
    }

    @Override
    public void periodic() {
        if (desiredGoal != currentGoal) {
            arm.setGoal(desiredGoal.armGoal);
            shooter.setGoal(desiredGoal.shooterGoal);
            this.currentGoal = desiredGoal;
        }

        Logger.recordOutput(LogKey + "/Goal", currentGoal.toString());
        Logger.recordOutput(LogKey + "/AtSetpoint", atSetpoint.getAsBoolean());
    }

    public Goal getGoal() {
        return desiredGoal;
    }

    public Command toInstantGoal(final Goal goal) {
        return Commands.runOnce(() -> this.desiredGoal = goal);
    }

    public Command toGoal(final Goal goal) {
        return Commands.runEnd(() -> this.desiredGoal = goal, () -> this.desiredGoal = Goal.IDLE);
    }

    public Command runGoal(final Goal goal) {
        return Commands.run(() -> this.desiredGoal = goal);
    }

    public Command toState(final Supplier<ShotParameters.Parameters> parametersSupplier) {
        return toState(
                () -> parametersSupplier.get().armPivotAngle(),
                () -> parametersSupplier.get().ampVelocityRotsPerSec(),
                () -> parametersSupplier.get().leftVelocityRotsPerSec(),
                () -> parametersSupplier.get().rightVelocityRotsPerSec()
        );
    }

    public Command toState(
            final Supplier<Rotation2d> armPivotPosition,
            final DoubleSupplier ampVelocityRotsPerSec,
            final DoubleSupplier leftVelocityRotsPerSec,
            final DoubleSupplier rightVelocityRotsPerSec
    ) {
        return Commands.parallel(
                Commands.runEnd(() -> this.desiredGoal = Goal.NONE, () -> this.desiredGoal = Goal.IDLE),
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
                Commands.run(() -> this.desiredGoal = Goal.NONE),
                arm.runPivotVoltageCommand(armPivotVolts),
                shooter.runVoltageCommand(
                        ampVolts,
                        leftVolts,
                        rightVolts
                )
        );
    }

    public Set<Subsystem> getRequirements() {
        return Set.of(arm, shooter);
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
