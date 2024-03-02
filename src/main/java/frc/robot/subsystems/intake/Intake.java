package frc.robot.subsystems.intake;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Current;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Time;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.constants.Constants;
import frc.robot.constants.HardwareConstants;
import frc.robot.utils.logging.LogUtils;
import org.littletonrobotics.junction.Logger;

import java.util.function.Supplier;

import static edu.wpi.first.units.Units.*;

public class Intake extends SubsystemBase {
    protected static final String logKey = "Intake";

    private static final double MaxRollerSurfaceSpeedMetersPerSec =
            Constants.Intake.RollerCircumferenceMeters * Constants.Swerve.ROBOT_MAX_SPEED_MPS;

    private final IntakeIO intakeIO;
    private final IntakeIOInputsAutoLogged inputs;
    private final Supplier<ChassisSpeeds> chassisSpeedsSupplier;

    private final SysIdRoutine torqueCurrentSysIdRoutine;

    public enum State {
        INTAKE,
        OUTTAKE,
        IDLE
    }

    public static class Setpoint {
        public double rightRollerVelocityRotsPerSecond;
        public double leftRollerVelocityRotsPerSecond;
        public double shooterFeederRollerVelocityRotsPerSecond;
    }

    public Intake(
            final Constants.RobotMode robotMode,
            final HardwareConstants.IntakeConstants intakeConstants,
            final Supplier<ChassisSpeeds> chassisSpeedsSupplier
    ) {
        this.intakeIO = switch (robotMode) {
            case REAL -> new IntakeIOReal(intakeConstants);
            case SIM -> new IntakeIOSim(intakeConstants);
            case REPLAY -> new IntakeIO() {};
        };

        this.chassisSpeedsSupplier = chassisSpeedsSupplier;

        this.inputs = new IntakeIOInputsAutoLogged();

        this.torqueCurrentSysIdRoutine = makeTorqueCurrentSysIdRoutine(
                Amps.of(2).per(Second),
                Amps.of(10),
                Seconds.of(10)
        );

        this.intakeIO.config();
        this.intakeIO.initialize();
    }

    @Override
    public void periodic() {
        final double intakeIOPeriodicStart = Logger.getRealTimestamp();
        intakeIO.updateInputs(inputs);
        Logger.processInputs(logKey, inputs);

        Logger.recordOutput(
                logKey + "/PeriodicIOPeriodMs",
                LogUtils.microsecondsToMilliseconds(Logger.getRealTimestamp() - intakeIOPeriodicStart)
        );

    }

    //TODO: this will eventually take an enum probably
    public Command setStateCommand(final State intakeState) {
        return run(() -> {
            final Setpoint setpoint = new Setpoint();
            switch (intakeState) {
                case INTAKE -> {
                    final ChassisSpeeds chassisSpeeds = chassisSpeedsSupplier.get();
                    final Translation2d chassisSpeedTranslation = new Translation2d(
                            chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond);

                    final double rollerSpeedRotsPerSec = MathUtil.clamp(
                            2 * chassisSpeedTranslation.getNorm(),
                            0.25 * MaxRollerSurfaceSpeedMetersPerSec,
                            MaxRollerSurfaceSpeedMetersPerSec
                    ) / Constants.Intake.RollerCircumferenceMeters;

                    setpoint.rightRollerVelocityRotsPerSecond = rollerSpeedRotsPerSec;
                    setpoint.leftRollerVelocityRotsPerSecond = rollerSpeedRotsPerSec;
                    setpoint.shooterFeederRollerVelocityRotsPerSecond = rollerSpeedRotsPerSec;
                }
                case OUTTAKE -> {
                    setpoint.rightRollerVelocityRotsPerSecond = -MaxRollerSurfaceSpeedMetersPerSec;
                    setpoint.leftRollerVelocityRotsPerSecond = -MaxRollerSurfaceSpeedMetersPerSec;
                    setpoint.shooterFeederRollerVelocityRotsPerSecond = -MaxRollerSurfaceSpeedMetersPerSec;
                }
                case IDLE -> {
                    setpoint.rightRollerVelocityRotsPerSecond = 0.1 * MaxRollerSurfaceSpeedMetersPerSec;
                    setpoint.leftRollerVelocityRotsPerSecond = 0.1 * MaxRollerSurfaceSpeedMetersPerSec;
                    setpoint.shooterFeederRollerVelocityRotsPerSecond = 0.1 * MaxRollerSurfaceSpeedMetersPerSec;
                }
            }

            Logger.recordOutput(logKey + "/RightRollerVelocityRotPerSec", setpoint.rightRollerVelocityRotsPerSecond);
            Logger.recordOutput(logKey + "/LeftRollerVelocityRotPerSec", setpoint.leftRollerVelocityRotsPerSecond);
            Logger.recordOutput(logKey + "/ShooterFeederRollerVelocityRotPerSec", setpoint.shooterFeederRollerVelocityRotsPerSecond);

            intakeIO.toVelocity(
                    setpoint.rightRollerVelocityRotsPerSecond,
                    setpoint.leftRollerVelocityRotsPerSecond,
                    setpoint.shooterFeederRollerVelocityRotsPerSecond
            );
        });
    }
    private SysIdRoutine makeTorqueCurrentSysIdRoutine(
            final Measure<Velocity<Current>> currentRampRate,
            final Measure<Current> stepCurrent,
            final Measure<Time> timeout
    ) {
        return new SysIdRoutine(
                new SysIdRoutine.Config(
                        // we need to lie to SysId here, because it only takes voltage instead of current
                        Volts.per(Second).of(currentRampRate.baseUnitMagnitude()),
                        Volts.of(stepCurrent.baseUnitMagnitude()),
                        timeout,
                        state -> SignalLogger.writeString("state", state.toString())
                ),
                new SysIdRoutine.Mechanism(
                        voltageMeasure -> intakeIO.toTorqueCurrent(
                                // this is really in amps, not volts
                                voltageMeasure.in(Volts),
                                voltageMeasure.in(Volts),
                                voltageMeasure.in(Volts)
                        ),
                        null,
                        this
                )
        );
    }

    public Command runSysIDRoutineTorqueCurrent() {
        return Commands.sequence(
                torqueCurrentSysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward),
                Commands.waitSeconds(4),
                torqueCurrentSysIdRoutine.quasistatic(SysIdRoutine.Direction.kReverse),
                Commands.waitSeconds(4),
                torqueCurrentSysIdRoutine.dynamic(SysIdRoutine.Direction.kForward),
                Commands.waitSeconds(4),
                torqueCurrentSysIdRoutine.dynamic(SysIdRoutine.Direction.kReverse),
                Commands.waitSeconds(4),
                Commands.runOnce(SignalLogger::stop)
        );
    }
}
