package frc.robot.utils.ctre;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.ParentDevice;
import edu.wpi.first.wpilibj.DriverStation;

public class Phoenix6Utils {
    /**
     * Performs latency compensation on a refreshed {@link StatusSignal}
     * (using {@link BaseStatusSignal#getLatencyCompensatedValue(StatusSignal, StatusSignal)})
     * <p>
     * Only compensates if the {@link StatusCode} of the {@link StatusSignal}
     * is OK ({@link StatusCode#isOK()})
     * <p>
     * If the {@link StatusCode} is not OK, then it just returns the signal value without latency compensation
     * @param refreshedSignal the {@link StatusSignal}, ex. Position
     * @param refreshedDeltaSignal the delta/derivative of the {@link StatusSignal}, ex. Velocity
     * @return the latency compensated value
     * @see StatusSignal
     * @see StatusCode
     * @see BaseStatusSignal#getLatencyCompensatedValue(StatusSignal, StatusSignal)
     */
    public static double latencyCompensateIfSignalIsGood(
            final StatusSignal<Double> refreshedSignal,
            final StatusSignal<Double> refreshedDeltaSignal
    ) {
        if (refreshedSignal.getStatus().isOK() && refreshedDeltaSignal.getStatus().isOK()) {
            return BaseStatusSignal.getLatencyCompensatedValue(
                    refreshedSignal,
                    refreshedDeltaSignal
            );
        } else {
            return refreshedSignal.getValue();
        }
    }
    /**
     * Exception thrown when a {@link StatusCode} assertion fails.
     * @see Phoenix6Utils#assertIsOK(StatusCode)
     */
    public static class StatusCodeAssertionException extends RuntimeException {
        public StatusCodeAssertionException(final StatusCode expected, final StatusCode got) {
            super(String.format("Expected StatusCode %s; got %s", expected.getName(), got.getName()));
        }

        public StatusCodeAssertionException(final StatusCode got) {
            this(StatusCode.OK, got);
        }
    }

    /**
     * Assert that a {@link StatusCode} must be {@link StatusCode#isOK()}.
     * @param statusCode the {@link StatusCode}
     * @throws StatusCodeAssertionException if the {@link StatusCode} is not {@link StatusCode#isOK()}
     */
    public static void assertIsOK(final StatusCode statusCode) {
        if (!statusCode.isOK()) {
            throw new StatusCodeAssertionException(statusCode);
        }
    }

    public static void reportIfNotOk(final ParentDevice parentDevice, final StatusCode statusCode) {
        if (!statusCode.isOK()) {
            DriverStation.reportError(
                    String.format(
                            "Failed on Device %d: %s",
                            parentDevice.getDeviceID(),
                            statusCode.getName()
                    ),
                    false
            );
        }
    }
}
