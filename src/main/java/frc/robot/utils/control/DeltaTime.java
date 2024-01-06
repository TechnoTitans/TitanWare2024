package frc.robot.utils.control;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.constants.Constants;
import org.littletonrobotics.junction.Logger;

public class DeltaTime {
    private static final double MICRO_TO_SEC = 1e-6;
    private final Timer timer;
    private final boolean disableLoggedTimestamps;
    private boolean isFirstCall = true;

    private double lastRealFPGASeconds;

    /**
     * Creates a new DeltaTime instance, with an option to disableDeterministicTimestamps
     * @param disableLoggedTimestamps should be true if {@link Logger#disableDeterministicTimestamps()} is called,
     *                                false if not
     * @see Logger#disableDeterministicTimestamps()
     */
    public DeltaTime(final boolean disableLoggedTimestamps) {
        this.timer = new Timer();
        this.disableLoggedTimestamps = disableLoggedTimestamps;
        this.lastRealFPGASeconds = MICRO_TO_SEC * Logger.getRealTimestamp();

        if (disableLoggedTimestamps) {
            DriverStation.reportWarning(
                    "LoggedTimestamps disabled! Replayed logic will no longer be deterministic!",
                    true
            );
        }
    }

    /**
     * Creates a new DeltaTime instance, with deterministic timestamps enabled
     * @see DeltaTime#DeltaTime(boolean)  DeltaTime
     */
    public DeltaTime() {
        this(false);
    }

    /**
     * Get the delta time (dt) since the last time {@link DeltaTime#get()} or equiv. was called,
     * with a custom default loop period (in seconds)
     *
     * @param orElseSeconds returns this dt (sec) if this is the first call to {@link DeltaTime#get()} or equiv.
     * @return the measured delta time (dt), in seconds
     * @implNote if disableLoggedTimestamps is true, this will report the true FPGA timestamp (from the HAL)
     */
    public double getOrElse(final double orElseSeconds) {
        if (isFirstCall) {
            isFirstCall = false;
            timer.start();
            return orElseSeconds;
        } else if (!disableLoggedTimestamps) {
            final double dtSeconds = timer.get();
            timer.reset();
            return dtSeconds;
        } else {
            final double lastFPGASeconds = lastRealFPGASeconds;
            final double newRealTimestampSeconds = MICRO_TO_SEC * Logger.getRealTimestamp();

            lastRealFPGASeconds = newRealTimestampSeconds;
            return newRealTimestampSeconds - lastFPGASeconds;
        }
    }

    /**
     * Get the delta time (dt) since the last time this method or equiv. was called
     * @return the measured delta time (dt), in seconds
     * @see DeltaTime#getOrElse(double)
     */
    public double get() {
        return getOrElse(Constants.LOOP_PERIOD_SECONDS);
    }
}
