package frc.robot.subsystems.superstructure;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.superstructure.arm.Arm;
import frc.robot.subsystems.superstructure.shooter.Shooter;

public class Superstructure {
    private final Arm arm;
    private final Shooter shooter;

    public record VelocitySetpoint(
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

    public Superstructure(final Arm arm, final Shooter shooter) {
        this.arm = arm;
        this.shooter = shooter;
    }

    public Command toVelocitySetpoint(final VelocitySetpoint velocitySetpoint) {
        return Commands.parallel(
                arm.toPivotPositionCommand(velocitySetpoint.armPivotPositionRots),
                shooter.toVelocityCommand(
                        velocitySetpoint.ampVelocityRotsPerSec,
                        velocitySetpoint.leftVelocityRotsPerSec,
                        velocitySetpoint.rightVelocityRotsPerSec
                )
        );
    }

    public Command toVoltageSetpoint(final VoltageSetpoint voltageSetpoint) {
        return Commands.parallel(
                shooter.toVoltageCommand(
                        voltageSetpoint.ampVolts,
                        voltageSetpoint.leftVolts,
                        voltageSetpoint.rightVolts
                )
        );
    }
}
