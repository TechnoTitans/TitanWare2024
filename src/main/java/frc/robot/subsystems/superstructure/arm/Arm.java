package frc.robot.subsystems.superstructure.arm;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.math.*;
import edu.wpi.first.math.controller.LinearPlantInversionFeedforward;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.constants.Constants;
import frc.robot.constants.HardwareConstants;
import frc.robot.constants.SimConstants;
import frc.robot.utils.logging.LogUtils;
import org.littletonrobotics.junction.Logger;

import java.util.function.DoubleSupplier;

import static edu.wpi.first.units.Units.*;

public class Arm extends SubsystemBase {
    protected static final String LogKey = "Arm";
    private static final double PositionToleranceRots = 0.005;
    private static final double VelocityToleranceRotsPerSec = 0.01;

    private final ArmIO armIO;
    private final ArmIOInputsAutoLogged inputs;

    private final SysIdRoutine voltageSysIdRoutine;

    private Goal goal = Goal.STOW;
    private Goal previousGoal = goal;

    private final PositionSetpoint setpoint;
    private final PositionSetpoint pivotSoftLowerLimit;
    private final PositionSetpoint pivotSoftUpperLimit;

    private final TrapezoidProfile profile;
    private TrapezoidProfile.State lastProfiledReference = new TrapezoidProfile.State(
            Units.rotationsToRadians(goal.pivotPositionGoal),
            0
    );

    private final LinearQuadraticRegulator<N2, N1, N1> controller;
    private final LinearPlantInversionFeedforward<N2, N1, N1> feedforward;
    private final KalmanFilter<N3, N1, N1> observer;

    public Trigger atPivotSetpoint = new Trigger(this::atPositionSetpoint);
    public Trigger atPivotLowerLimit = new Trigger(this::atPivotLowerLimit);
    public Trigger atPivotUpperLimit = new Trigger(this::atPivotUpperLimit);
    public Trigger atPivotLimit = atPivotLowerLimit.or(atPivotUpperLimit);

    public static class PositionSetpoint {
        public double pivotPositionRots = 0;

        public PositionSetpoint withPivotPositionRots(final double pivotPositionRots) {
            this.pivotPositionRots = pivotPositionRots;
            return this;
        }

        public boolean atSetpoint(final double pivotPositionRots, final double pivotVelocityRotsPerSec) {
            return MathUtil.isNear(this.pivotPositionRots, pivotPositionRots, PositionToleranceRots)
                    && MathUtil.isNear(0, pivotVelocityRotsPerSec, VelocityToleranceRotsPerSec);
        }
    }

    public enum Goal {
        NONE(0),
        ZERO(0),
        STOW(Units.degreesToRotations(10)),
        AMP(Units.degreesToRotations(91)),
        TEST(Units.degreesToRotations(19)),
        SUBWOOFER(Units.degreesToRotations(56.5));

        private final double pivotPositionGoal;
        Goal(final double pivotPositionGoal) {
            this.pivotPositionGoal = pivotPositionGoal;
        }

        public double getPivotPositionGoal() {
            return pivotPositionGoal;
        }
    }

    private final Vector<N3> RotationAxis = VecBuilder.fill(0, 1, 0);
    private final Pose3d RootPose = new Pose3d().transformBy(SimConstants.Arm.ROBOT_TO_PIVOT_TRANSFORM);

    public Arm(final Constants.RobotMode mode, final HardwareConstants.ArmConstants armConstants) {
        this.armIO = switch (mode) {
            case REAL -> new ArmIOReal(armConstants);
            case SIM -> new ArmIOSim(armConstants);
            case REPLAY -> new ArmIO() {};
        };

        this.profile = new TrapezoidProfile(
                new TrapezoidProfile.Constraints(
                        Units.degreesToRadians(60),
                        Units.degreesToRadians(120)
                )
        );

        final double gearing = armConstants.pivotGearing();
        final double momentOfInertia = armConstants.pivotMomentOfInertia();
        final DCMotor motorModel = DCMotor.getKrakenX60Foc(2);
        final LinearSystem<N2, N1, N1> singleJointedPlant = LinearSystemId.createSingleJointedArmSystem(
                motorModel,
                momentOfInertia,
                gearing
        );

        this.controller = new LinearQuadraticRegulator<>(
                singleJointedPlant,
                VecBuilder.fill(Units.degreesToRadians(1.975), Units.degreesToRadians(12.5)),
                VecBuilder.fill(12),
                Constants.LOOP_PERIOD_SECONDS
        );
        this.feedforward = new LinearPlantInversionFeedforward<>(singleJointedPlant, 0.02);


        final LinearSystem<N3, N1, N1> plant = new LinearSystem<>(
                MatBuilder.fill(
                        Nat.N3(),
                        Nat.N3(),
                        0.0,
                        1.0,
                        0.0,
                        0.0,
                        -Math.pow(gearing, -2) * motorModel.KtNMPerAmp /
                                (motorModel.KvRadPerSecPerVolt * motorModel.rOhms * momentOfInertia),
                        Math.pow(gearing, -1) * motorModel.KtNMPerAmp /
                                (motorModel.rOhms * momentOfInertia),
                        0.0,
                        0.0,
                        0.0
                ),
                MatBuilder.fill(
                        Nat.N3(),
                        Nat.N1(),
                        0.0,
                        Math.pow(gearing, -1.0) * motorModel.KtNMPerAmp / (motorModel.rOhms * momentOfInertia),
                        0.0
                ),
                MatBuilder.fill(
                        Nat.N1(),
                        Nat.N3(),
                        1.0,
                        0.0,
                        0.0
                ),
                new Matrix<>(Nat.N1(), Nat.N1())
        );

        this.observer = new KalmanFilter<>(
                Nat.N3(),
                Nat.N1(),
                plant,
                VecBuilder.fill(
                        Units.degreesToRadians(10),
                        Units.degreesToRadians(20),
                        0.085
                ),
                VecBuilder.fill(0.175),
                Constants.LOOP_PERIOD_SECONDS
        );

        this.inputs = new ArmIOInputsAutoLogged();
        this.voltageSysIdRoutine = makeVoltageSysIdRoutine(
                Volts.of(4).per(Second),
                Volts.of(8),
                Seconds.of(6)
        );

        this.setpoint = new PositionSetpoint();
        this.pivotSoftLowerLimit = new PositionSetpoint()
                .withPivotPositionRots(armConstants.pivotSoftLowerLimitRots());
        this.pivotSoftUpperLimit = new PositionSetpoint()
                .withPivotPositionRots(armConstants.pivotSoftUpperLimitRots());

        this.armIO.config();
        this.init();
    }

    private Pose3d armPoseFromAngle(final double angleRads) {
        return RootPose.transformBy(
                new Transform3d(
                        SimConstants.Arm.PIVOT_SHAFT_TO_CENTER_TRANSFORM
                                .rotateBy(new Rotation3d(RotationAxis, angleRads)),
                        new Rotation3d(0, angleRads, 0)
                )
        );
    }

    private void init() {
        controller.reset();
        feedforward.reset(VecBuilder.fill(lastProfiledReference.position, lastProfiledReference.velocity));
        observer.setXhat(VecBuilder.fill(lastProfiledReference.position, lastProfiledReference.velocity, 0));
    }

    private double getVoltageInput() {
        return controller.getU(0) + feedforward.getUff(0) - observer.getXhat(2);
    }

    private void correct() {
        observer.correct(
                VecBuilder.fill(getVoltageInput()),
                VecBuilder.fill(Units.rotationsToRadians(inputs.leftPivotPositionRots))
        );
    }

    private void predict() {
        final Matrix<N1, N1> voltage = controller.calculate(
                VecBuilder.fill(observer.getXhat(0), observer.getXhat(1)),
                VecBuilder.fill(lastProfiledReference.position, lastProfiledReference.velocity)
        ).plus(
                feedforward.calculate(
                        VecBuilder.fill(lastProfiledReference.position, lastProfiledReference.velocity)
                )
        ).plus(
                -observer.getXhat(2)
        );

        observer.predict(voltage, Constants.LOOP_PERIOD_SECONDS);
    }

    @Override
    public void periodic() {
        final double armPeriodicUpdateStart = Logger.getRealTimestamp();

        armIO.updateInputs(inputs);
        Logger.processInputs(LogKey, inputs);
        Logger.recordOutput(
                LogKey + "/PeriodicIOPeriodMs",
                LogUtils.microsecondsToMilliseconds(Logger.getRealTimestamp() - armPeriodicUpdateStart)
        );

        if (goal != Goal.NONE && previousGoal != goal) {
            setpoint.pivotPositionRots = goal.getPivotPositionGoal();
            this.previousGoal = goal;
        } else if (goal == Goal.NONE) {
            this.previousGoal = Goal.NONE;
        }

        lastProfiledReference = profile.calculate(
                Constants.LOOP_PERIOD_SECONDS,
                lastProfiledReference,
                new TrapezoidProfile.State(Units.rotationsToRadians(setpoint.pivotPositionRots), 0)
        );

        correct();
        predict();
        armIO.toPivotVoltage(getVoltageInput());

        Logger.recordOutput(LogKey + "/Goal", goal.toString());
        Logger.recordOutput(LogKey + "/PositionSetpoint/PivotPositionRots", setpoint.pivotPositionRots);
        Logger.recordOutput(LogKey + "/AtPositionSetpoint", atPositionSetpoint());
        Logger.recordOutput(LogKey + "/AtPivotLowerLimit", atPivotLowerLimit());
        Logger.recordOutput(LogKey + "/AtPivotUpperLimit", atPivotUpperLimit());

        Logger.recordOutput(
                LogKey + "/Pose",
                armPoseFromAngle(Units.rotationsToRadians(-inputs.leftPivotPositionRots))
        );

        Logger.recordOutput(
                LogKey + "/GoalPose",
                armPoseFromAngle(Units.rotationsToRadians(-setpoint.pivotPositionRots))
        );
    }

    private boolean atPositionSetpoint() {
        return setpoint.atSetpoint(inputs.leftPivotPositionRots, inputs.leftPivotVelocityRotsPerSec);
    }

    private boolean atPivotLowerLimit() {
        return inputs.leftPivotPositionRots <= pivotSoftLowerLimit.pivotPositionRots;
    }

    private boolean atPivotUpperLimit() {
        return inputs.leftPivotPositionRots >= pivotSoftUpperLimit.pivotPositionRots;
    }

    public Command toGoal(final Goal goal) {
        // TODO: need to standardize on using runOnce vs. runEnd, i.e. whether this command,
        //  on end/interrupt should schedule the default/idle goal (in this case, STOW)
        return runEnd(() -> this.goal = goal, () -> this.goal = Goal.STOW);
    }

    public Command toPivotPositionCommand(final DoubleSupplier pivotPositionRots) {
        return Commands.sequence(
                Commands.runOnce(() -> this.goal = Goal.NONE),
                run(() -> setpoint.pivotPositionRots = pivotPositionRots.getAsDouble())
        );
    }

    public Command runPivotVoltageCommand(final double pivotVoltageVolts) {
        return run(() -> armIO.toPivotVoltage(pivotVoltageVolts));
    }

    @SuppressWarnings("unused")
    public Command homePivotWithCurrentCommand() {
        return Commands.sequence(
                startEnd(
                        () -> armIO.toPivotVoltage(-1),
                        () -> armIO.toPivotVoltage(0)
                ).until(() -> inputs.leftPivotTorqueCurrentAmps >= 25 || inputs.rightPivotTorqueCurrentAmps >= 25),
                runOnce(() -> {
                    armIO.setPivotPosition(0);
                    armIO.configureSoftLimits(pivotSoftLowerLimit, pivotSoftUpperLimit);
                })
        );
    }

    private SysIdRoutine makeVoltageSysIdRoutine(
            final Measure<Velocity<Voltage>> voltageRampRate,
            final Measure<Voltage> stepVoltage,
            final Measure<Time> timeout
    ) {
        return new SysIdRoutine(
                new SysIdRoutine.Config(
                        voltageRampRate,
                        stepVoltage,
                        timeout,
                        state -> SignalLogger.writeString(String.format("%s-state", LogKey), state.toString())
                ),
                new SysIdRoutine.Mechanism(
                        voltageMeasure -> armIO.toPivotVoltage(voltageMeasure.in(Volts)),
                        null,
                        this
                )
        );
    }

    private Command makeSysIdCommand(final SysIdRoutine sysIdRoutine) {
        return Commands.sequence(
                sysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward)
                        .until(atPivotUpperLimit),
                Commands.waitSeconds(4),
                sysIdRoutine.quasistatic(SysIdRoutine.Direction.kReverse)
                        .until(atPivotLowerLimit),
                Commands.waitSeconds(4),
                sysIdRoutine.dynamic(SysIdRoutine.Direction.kForward)
                        .until(atPivotUpperLimit),
                Commands.waitSeconds(4),
                sysIdRoutine.dynamic(SysIdRoutine.Direction.kReverse)
                        .until(atPivotLowerLimit)
        );
    }

    @SuppressWarnings("unused")
    public Command voltageSysIdCommand() {
        return makeSysIdCommand(voltageSysIdRoutine);
    }
}
