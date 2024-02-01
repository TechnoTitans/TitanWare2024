package frc.robot.utils.logging;

import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.Arguments;
import org.junit.jupiter.params.provider.MethodSource;

import java.util.stream.Stream;

import static org.junit.jupiter.api.Assertions.assertEquals;

class LogUtilsTest {
    public static final double EPSILON = 1E-8;

    @ParameterizedTest
    @MethodSource("provideMicrosecondsToMilliseconds")
    void microsecondsToMilliseconds(final double microsecond, final double expectMillisecond) {
        assertEquals(LogUtils.microsecondsToMilliseconds(microsecond), expectMillisecond, EPSILON);
    }

    private static Stream<Arguments> provideMicrosecondsToMilliseconds() {
        return Stream.of(
                Arguments.of(1.0, 0.001),
                Arguments.of(10.0, 0.01),
                Arguments.of(1e+6, 1e+3),
                Arguments.of(1e-8, 1e-11),
                Arguments.of(8912.724761, 8.912724761)
        );
    }
}