package frc.robot.utils.logging;

public class LogUtils {
    public static final double MICRO_TO_MILLI = 1d / 1000;

    public static double microsecondsToMilliseconds(final double microseconds) {
        return microseconds * MICRO_TO_MILLI;
    }
}
