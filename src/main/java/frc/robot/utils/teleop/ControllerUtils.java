package frc.robot.utils.teleop;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class ControllerUtils {
    public final static double STICK_TO_GYRO_ROTATION = -1;
    public final static Rotation2d STICK_TO_FIELD_RELATIVE_ROTATION = Rotation2d.fromDegrees(-90);

    /**
     * Converts an X & Y joystick input into a {@link Translation2d} describing the <b>squared</b> input of the stick.
     * <p>This gives the joystick (driver) finer control at lower speeds and is pretty standard practice in FRC.</p>
     * <p>
     *     See
     *     <a href="https://docs.wpilib.org/en/stable/docs/software/hardware-apis/motors/wpi-drive-classes.html#squaring-inputs">
     *         this page
     *     </a>
     *     about squared inputs.
     * </p>
     * @param xInput the xInput (typically your {@link XboxController#getLeftY()}, yes, <b>Y</b>)
     * @param yInput the yInput (typically your {@link XboxController#getLeftX()}, yes, <b>X</b>)
     * @param deadband the deadband
     * @return the {@link Translation2d} that describes the stick input, with deadband, scalar, sensitivity
     * and weight all applied
     *
     * @implNote This implementation of squared inputs is a bit trickier for swerve compared to differential drives
     * since one joystick is controlling two axes at once (X and Y). We square the magnitude of the joystick vector
     * rather than the individual axes, because otherwise the squaring will change the direction of travel.
     */
    public static Translation2d calculateLinearVelocity(
            final double xInput,
            final double yInput,
            final double deadband
    ) {
        final double magnitude = MathUtil.applyDeadband(Math.hypot(xInput, yInput), deadband);
        final Rotation2d direction = new Rotation2d(xInput, yInput);

        final double squaredMagnitude = Math.copySign(magnitude * magnitude, magnitude);
        return new Pose2d(new Translation2d(), direction)
                .transformBy(new Transform2d(squaredMagnitude, 0, new Rotation2d()))
                .getTranslation();
    }

    public static double getStickSquaredInput(final double input, final double deadband) {
        return MathUtil.applyDeadband(Math.copySign(input * input, input), deadband);
    }

    public static Command rumbleForDurationCommand(
            final GenericHID controller,
            final GenericHID.RumbleType rumbleType,
            final double value,
            final double timeSeconds
    ) {
        return Commands.sequence(
                Commands.runOnce(() -> controller.setRumble(rumbleType, value)),
                Commands.waitSeconds(timeSeconds),
                Commands.runOnce(() -> controller.setRumble(rumbleType, 0))
        );
    }
}
