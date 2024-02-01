package frc.robot.constants;

public interface SimConstants {
    // Assume 2mOhm resistance for voltage drop calculation
    double FALCON_MOTOR_RESISTANCE = 0.002;

    interface CTRE {
        // TODO: verify that config calls in sim simply just take longer,
        //  and thus need a longer timeout than 0.05s (50ms)
        boolean DISABLE_NEUTRAL_MODE_IN_SIM = false;
    }
}
