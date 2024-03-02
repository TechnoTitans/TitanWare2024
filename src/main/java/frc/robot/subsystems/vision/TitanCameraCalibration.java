package frc.robot.subsystems.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N5;
import org.photonvision.simulation.SimCameraProperties;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

@SuppressWarnings("UnusedReturnValue")
public class TitanCameraCalibration {
    private final Map<CameraProperties.Resolution, SimCameraProperties> cameraPropertiesMap;

    public TitanCameraCalibration(final Map<CameraProperties.Resolution, SimCameraProperties> cameraPropertiesMap) {
        this.cameraPropertiesMap = cameraPropertiesMap;
    }

    public TitanCameraCalibration() {
        this(new HashMap<>());
    }

    public static double getPxAvgError(final List<Double> pxErrors) {
        return pxErrors.stream().mapToDouble(error -> error).average().orElse(0);
    }

    public static TitanCameraCalibration fromSimCameraProperties(final SimCameraProperties simCameraProperties) {
        final int width = simCameraProperties.getResWidth();
        final int height = simCameraProperties.getResHeight();

        final CameraProperties.Resolution resolution =
                CameraProperties.Resolution
                        .getFromWidthHeight(width, height)
                        .orElseThrow(() -> new RuntimeException(
                                String.format(
                                        "Failed to find resolution of %s",
                                        CameraProperties.Resolution.rawToString(width, height)
                                )
                        ));

        return new TitanCameraCalibration(new HashMap<>(Map.of(resolution, simCameraProperties)));
    }

    public SimCameraProperties getOrMake(final CameraProperties.Resolution resolution) {
        final SimCameraProperties simCameraProperties = cameraPropertiesMap.get(resolution);
        if (simCameraProperties != null) {
            return simCameraProperties;
        }

        final SimCameraProperties newSimCameraProperties = new SimCameraProperties();
        cameraPropertiesMap.put(resolution, newSimCameraProperties);

        return newSimCameraProperties;
    }

    public TitanCameraCalibration withIntrinsics(
            final CameraProperties.Resolution resolution,
            final Matrix<N3, N3> intrinsics
    ) {
        final SimCameraProperties simCameraProperties = getOrMake(resolution);
        simCameraProperties.setCalibration(
                resolution.getWidth(), resolution.getHeight(), intrinsics, simCameraProperties.getDistCoeffs()
        );

        return this;
    }

    public TitanCameraCalibration withDistortions(
            final CameraProperties.Resolution resolution,
            final Matrix<N5, N1> distortions
    ) {
        final SimCameraProperties simCameraProperties = getOrMake(resolution);
        simCameraProperties.setCalibration(
                resolution.getWidth(), resolution.getHeight(), simCameraProperties.getIntrinsics(), distortions
        );

        return this;
    }

    public TitanCameraCalibration withIntrinsicsAndDistortions(
            final CameraProperties.Resolution resolution,
            final Matrix<N3, N3> intrinsics,
            final Matrix<N5, N1> distortions
    ) {
        withIntrinsics(resolution, intrinsics);
        withDistortions(resolution, distortions);

        return this;
    }

    public TitanCameraCalibration withPxErrorsAndStdDev(
            final CameraProperties.Resolution resolution,
            final List<Double> pxErrors,
            final double pxErrorStdDev
    ) {
        final SimCameraProperties simCameraProperties = getOrMake(resolution);
        simCameraProperties.setCalibError(getPxAvgError(pxErrors), pxErrorStdDev);

        return this;
    }

    public TitanCameraCalibration withCalibrations(
            final CameraProperties.Resolution resolution,
            final Matrix<N3, N3> intrinsics,
            final Matrix<N5, N1> distortions,
            final List<Double> pxErrors,
            final double pxErrorStdDev
    ) {
        withIntrinsicsAndDistortions(resolution, intrinsics, distortions);
        withPxErrorsAndStdDev(resolution, pxErrors, pxErrorStdDev);

        return this;
    }

    public boolean hasResolution(final CameraProperties.Resolution resolution) {
        return cameraPropertiesMap.containsKey(resolution);
    }

    public SimCameraProperties getSimCameraProperties(final CameraProperties.Resolution resolution) {
        return cameraPropertiesMap.get(resolution);
    }

    public Matrix<N3, N3> getIntrinsics(final CameraProperties.Resolution resolution) {
        return getSimCameraProperties(resolution).getIntrinsics();
    }

    public Matrix<N5, N1> getDistortions(final CameraProperties.Resolution resolution) {
        return getSimCameraProperties(resolution).getDistCoeffs();
    }
}
