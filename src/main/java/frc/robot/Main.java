package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

public final class Main {
    private Main() {}

    public static void main(final String... args) {
        RobotBase.startRobot(Robot::new);
    }
}
