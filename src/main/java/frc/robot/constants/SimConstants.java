package frc.robot.constants;

public interface SimConstants {
    // Assume 2mOhm resistance for voltage drop calculation
    double FALCON_MOTOR_RESISTANCE = 0.002;

    interface CTRE {
        boolean DISABLE_NEUTRAL_MODE_IN_SIM = true;
    }
}
