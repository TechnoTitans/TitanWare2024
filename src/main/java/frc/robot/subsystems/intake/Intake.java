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
import static frc.robot.constants.Constants.Intake.*;

public class Intake extends SubsystemBase {
    protected static final String logKey = "Intake";

    private static final double MaxRollerSurfaceSpeedMetersPerSec = Constants.Swerve.ROBOT_MAX_SPEED_MPS;

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
        public double frontRollersVelocityRotsPerSecond;
        public double backRollersVelocityRotsPerSecond;
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
                    ) / RollerCircumferenceMeters;

                    setpoint.frontRollersVelocityRotsPerSecond = rollerSpeedRotsPerSec;
                    setpoint.backRollersVelocityRotsPerSecond = rollerSpeedRotsPerSec;
                    setpoint.shooterFeederRollerVelocityRotsPerSecond = rollerSpeedRotsPerSec;
                }
                case OUTTAKE -> {
                    setpoint.frontRollersVelocityRotsPerSecond = -MaxRollerSurfaceSpeedMetersPerSec;
                    setpoint.backRollersVelocityRotsPerSecond = -MaxRollerSurfaceSpeedMetersPerSec;
                    setpoint.shooterFeederRollerVelocityRotsPerSecond = -MaxRollerSurfaceSpeedMetersPerSec;
                }
                case IDLE -> {
                    setpoint.frontRollersVelocityRotsPerSecond = 0.1 * MaxRollerSurfaceSpeedMetersPerSec;
                    setpoint.backRollersVelocityRotsPerSecond = 0.1 * MaxRollerSurfaceSpeedMetersPerSec;
                    setpoint.shooterFeederRollerVelocityRotsPerSecond = 0.1 * MaxRollerSurfaceSpeedMetersPerSec;
                }
            }

            Logger.recordOutput(logKey + "/FrontRollerVelocityRotPerSec", setpoint.frontRollersVelocityRotsPerSecond);
            Logger.recordOutput(logKey + "/BackRollerVelocityRotPerSec", setpoint.backRollersVelocityRotsPerSecond);
            Logger.recordOutput(logKey + "/ShooterFeederRollerVelocityRotPerSec", setpoint.shooterFeederRollerVelocityRotsPerSecond);

            intakeIO.toVelocity(
                    setpoint.frontRollersVelocityRotsPerSecond,
                    setpoint.frontRollersVelocityRotsPerSecond,
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
                        voltageMeasure -> intakeIO.setCharacterizationTorqueCurrent(
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

    @SuppressWarnings("unused")
    public Command torqueCurrentSysIdQuasistaticTestCommand(final SysIdRoutine.Direction direction) {
        return torqueCurrentSysIdRoutine.quasistatic(direction);
    }

    @SuppressWarnings("unused")
    public Command torqueCurrentSysIdDynamicTestCommand(final SysIdRoutine.Direction direction) {
        return torqueCurrentSysIdRoutine.dynamic(direction);
    }
}
