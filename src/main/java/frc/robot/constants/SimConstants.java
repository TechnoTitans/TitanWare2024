package frc.robot.constants;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public interface SimConstants {
    // Assume 2mOhm resistance for voltage drop calculation
    double FALCON_MOTOR_RESISTANCE = 0.002;

    interface CTRE {
        // TODO: verify that config calls in sim simply just take longer,
        //  and thus need a longer timeout than 0.05s (50ms)
        boolean DISABLE_NEUTRAL_MODE_IN_SIM = false;
    }

    interface Arm {
        double LENGTH_METERS = 0.508;

        Transform3d ROBOT_TO_PIVOT_TRANSFORM = new Transform3d(
                new Translation3d(Units.inchesToMeters(-6.00), 0, Units.inchesToMeters(11.71992391)),
                new Rotation3d(0, 0, 0)
        );

        Translation3d PIVOT_SHAFT_TO_CENTER_TRANSFORM = new Translation3d(
                Units.inchesToMeters(9.07063036), 0, Units.inchesToMeters(4.36006778)
        );
    }
}
