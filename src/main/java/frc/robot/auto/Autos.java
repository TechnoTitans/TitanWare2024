package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Swerve;

public class Autos {
    private final Swerve swerve;

    public Autos(final Swerve swerve) {
        this.swerve = swerve;
    }

    public Command sourceBoth() {
        return Commands.none();
    }
}
