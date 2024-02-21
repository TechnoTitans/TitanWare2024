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
//        return shooter.toVelocityCommand(
//                10,
//                10,
//                10
//        );
        return shooter.toVoltageCommand(
                0,
                10,
                10
        );
    }
}
