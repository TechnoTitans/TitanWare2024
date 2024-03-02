package frc.robot.subsystems.superstructure;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.superstructure.arm.Arm;
import frc.robot.subsystems.superstructure.shooter.Shooter;

public class Superstructure {
    private final Arm arm;
    private final Shooter shooter;

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

    public Superstructure(final Arm arm, final Shooter shooter) {
        this.arm = arm;
        this.shooter = shooter;
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
