package frc.robot.utils.sim;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.EnumSource;
import org.mockito.Spy;
import org.mockito.junit.jupiter.MockitoExtension;
import testutils.JNIUtils;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.mockito.Mockito.verify;

@ExtendWith(MockitoExtension.class)
class SimUtilsTest {

    @Spy
    private final TalonFX talonFX = new TalonFX(0);
    private final TalonFXSimState talonFXSimState = talonFX.getSimState();

    @Spy
    private final CANcoder canCoder = new CANcoder(0);
    private final CANcoderSimState cancoderSimState = canCoder.getSimState();

    @BeforeAll
    static void beforeAll() {
        JNIUtils.initializeHAL();
        JNIUtils.loadCTREPhoenix6JNI();
    }

    @Test
    void invertedValueToChassisReference() {
        assertEquals(
                ChassisReference.CounterClockwise_Positive,
                SimUtils.invertedValueToChassisReference(InvertedValue.CounterClockwise_Positive)
        );

        assertEquals(
                ChassisReference.Clockwise_Positive,
                SimUtils.invertedValueToChassisReference(InvertedValue.Clockwise_Positive)
        );
    }

    @Test
    void sensorDirectionToChassisReference() {
        assertEquals(
                ChassisReference.CounterClockwise_Positive,
                SimUtils.sensorDirectionToChassisReference(SensorDirectionValue.CounterClockwise_Positive)
        );

        assertEquals(
                ChassisReference.Clockwise_Positive,
                SimUtils.sensorDirectionToChassisReference(SensorDirectionValue.Clockwise_Positive)
        );
    }

    @ParameterizedTest
    @EnumSource(InvertedValue.class)
    void setCTRETalonFXSimStateMotorInverted(final InvertedValue inverted) {
        SimUtils.setCTRETalonFXSimStateMotorInverted(talonFX, inverted);
        assertEquals(SimUtils.invertedValueToChassisReference(inverted), talonFXSimState.Orientation);
    }

    @ParameterizedTest
    @EnumSource(SensorDirectionValue.class)
    void setCTRECANCoderSimStateSensorDirection(final SensorDirectionValue sensorDirectionValue) {
        SimUtils.setCTRECANCoderSimStateSensorDirection(canCoder, sensorDirectionValue);
        assertEquals(SimUtils.sensorDirectionToChassisReference(sensorDirectionValue), cancoderSimState.Orientation);
    }

    @Test
    void initializeCTRECANCoderSim() {
        // cannot mock static native methods, so cannot test that an exception will be thrown when NOT IN simulation
        // however, we can still test that the setPosition call goes through while IN simulation
        SimUtils.initializeCTRECANCoderSim(canCoder);
        verify(canCoder).setPosition(0);
    }
}