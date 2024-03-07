package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.constants.Constants;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.HardwareConstants;
import frc.robot.constants.RobotMap;
import frc.robot.subsystems.drive.Swerve;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.superstructure.ShotParameters;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.superstructure.arm.Arm;
import frc.robot.subsystems.superstructure.shooter.Shooter;
import frc.robot.subsystems.vision.PhotonVision;

import java.util.function.Supplier;

public class RobotContainer {
    public final PowerDistribution powerDistribution;

    public final Swerve swerve;
    public final Intake intake;

    public final Arm arm;
    public final Shooter shooter;

    public final Superstructure superstructure;

    public final PhotonVision photonVision;

    public final CommandXboxController driverController;
    public final CommandXboxController coDriverController;

    public RobotContainer() {
        this.powerDistribution = new PowerDistribution(
                RobotMap.PowerDistributionHub,
                PowerDistribution.ModuleType.kRev
        );
        this.powerDistribution.clearStickyFaults();
        this.powerDistribution.setSwitchableChannel(true);

        this.swerve = new Swerve(
                Constants.CURRENT_MODE,
                HardwareConstants.GYRO,
                HardwareConstants.FRONT_LEFT_MODULE,
                HardwareConstants.FRONT_RIGHT_MODULE,
                HardwareConstants.BACK_LEFT_MODULE,
                HardwareConstants.BACK_RIGHT_MODULE
        );

        this.intake = new Intake(
                Constants.CURRENT_MODE,
                HardwareConstants.INTAKE
        );

        this.arm = new Arm(Constants.RobotMode.REPLAY, HardwareConstants.ARM);
        this.shooter = new Shooter(Constants.CURRENT_MODE, HardwareConstants.SHOOTER);

        this.superstructure = new Superstructure(arm, shooter);

        this.photonVision = new PhotonVision(Constants.CURRENT_MODE, swerve, swerve.getPoseEstimator());

        this.driverController = new CommandXboxController(RobotMap.MainController);
        this.coDriverController = new CommandXboxController(RobotMap.CoController);

        this.driverController.y().onTrue(
                Commands.sequence(
                        superstructure.toGoal(Superstructure.Goal.IDLE),
                        Commands.waitUntil(superstructure.atGoalTrigger),
                        Commands.waitSeconds(6),
                        superstructure.toGoal(Superstructure.Goal.SUBWOOFER),
                        Commands.waitUntil(superstructure.atGoalTrigger),
                        Commands.waitSeconds(4),
                        superstructure.toGoal(Superstructure.Goal.IDLE)
                )
        );
    }

    public Command teleopDriveAimAndShoot() {
        final double rotationToleranceRadians = Units.degreesToRadians(3);
        final ProfiledPIDController rotationController = new ProfiledPIDController(
                5, 0, 0,
                new TrapezoidProfile.Constraints(
                        Constants.Swerve.ROBOT_MAX_ANGULAR_SPEED_RAD_PER_SEC * 0.75,
                        Constants.Swerve.TELEOP_MAX_ANGULAR_SPEED_RAD_PER_SEC * 0.5
                )
        );

        final Supplier<Rotation2d> pointAtSpeakerRotationSupplier =
                () -> swerve.getEstimatedPosition()
                        .getTranslation()
                        .minus(FieldConstants.getSpeakerPose().getTranslation())
                        .getAngle();

        //noinspection SuspiciousNameCombination
        return Commands.sequence(
                Commands.runOnce(() -> rotationController.reset(
                        swerve.getYaw().getRadians(),
                        swerve.getFieldRelativeSpeeds().omegaRadiansPerSecond)
                ),
                Commands.parallel(
                        swerve.teleopDriveFacingAngleCommand(
                                rotationController,
                                driverController::getLeftY,
                                driverController::getLeftX,
                                pointAtSpeakerRotationSupplier
                        ),
                        superstructure.toState(() -> ShotParameters.get(
                                swerve.getEstimatedPosition()
                                        .minus(FieldConstants.getSpeakerPose())
                                        .getTranslation()
                                        .getNorm())
                        ),
                        intake
                                .stopCommand()
                                .until(superstructure.atGoalTrigger.and(
                                        () -> MathUtil.isNear(
                                                pointAtSpeakerRotationSupplier.get().getRadians(),
                                                swerve.getEstimatedPosition().getRotation().getRadians(),
                                                rotationToleranceRadians

                                )))
                                .andThen(intake.feedCommand())
                )
        );
    }

    public Command getAutonomousCommand() {
        return intake.outtakeCommand();
    }
}
