package testutils;

import com.ctre.phoenix6.jni.CtreJniWrapper;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.util.RuntimeLoader;
import frc.robot.utils.closeables.ToClose;

import java.io.IOException;

import static org.junit.jupiter.api.Assertions.assertTrue;

public class JNIUtils {
    private static boolean runningHAL = false;
    static {
        if (!ToClose.hasHooked() && !ToClose.hasClosed()) {
            ToClose.hook();
        }
    }

    public static void initializeHAL() {
        if (runningHAL) {
            return;
        }

        assertTrue(HAL.initialize(500, 0));
        ToClose.add(JNIUtils::shutdownHAL);

        runningHAL = true;
    }

    public static void shutdownHAL() {
        if (runningHAL) {
            HAL.shutdown();
            runningHAL = false;
        }
    }

    public static void loadCTREPhoenix6JNI() {
        try {
            final RuntimeLoader<CtreJniWrapper> jniLoader = new RuntimeLoader<>(
                    "CTRE_PhoenixTools_Sim", RuntimeLoader.getDefaultExtractionRoot(), CtreJniWrapper.class
            );
            jniLoader.loadLibrary();
        } catch (final IOException ioException) {
            throw new RuntimeException(ioException);
        }
    }
}
