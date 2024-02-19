package frc.robot.subsystems.superstructure;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.superstructure.arm.Arm;
import frc.robot.subsystems.superstructure.shooter.Shooter;

public class Superstructure {
    private final Arm arm;
    private final Shooter shooter;

    public Superstructure(final Arm arm, final Shooter shooter) {
        this.arm = arm;
        this.shooter = shooter;
    }

    public Command cook() {
        return Commands.sequence(
                arm.toPivotPositionCommand(0.25),
                shooter.toVelocityCommand(85, 85),
                Commands.waitSeconds(0.5),
                shooter.toVelocityCommand(0, 0),
                arm.toPivotPositionCommand(0)
        );
    }
}
