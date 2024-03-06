package frc.robot.subsystems.vision;

import java.util.*;
import java.util.stream.Collectors;

public enum CameraProperties {
    PERFECT_90(
            Resolution.R960x720, 90, 80
    );

    private final List<Resolution> resolutions;
    private final double camDiagonalFOVDeg;
    private final double avgFPS;

    CameraProperties(
            final List<Resolution> resolutions,
            final double camDiagonalFOVDeg,
            final double avgFPS
    ) {
        if (resolutions.isEmpty()) {
            throw new IllegalArgumentException("Cannot provide a 0-sized resolution list!");
        }

        this.resolutions = resolutions;
        this.camDiagonalFOVDeg = camDiagonalFOVDeg;
        this.avgFPS = avgFPS;
    }

    CameraProperties(
            final Resolution resolution,
            final double camDiagonalFOVDeg,
            final double avgFPS
    ) {
        this(List.of(resolution), camDiagonalFOVDeg, avgFPS);
    }

    @SuppressWarnings("unused")
    CameraProperties(
            final int camResolutionWidthPx,
            final int camResolutionHeightPx,
            final double camDiagonalFOVDeg,
            final double avgFPS
    ) {
        this(
                Resolution
                        .getFromWidthHeight(camResolutionWidthPx, camResolutionHeightPx)
                        .orElseThrow(
                                () -> new RuntimeException(
                                        String.format(
                                                "Failed to find %s",
                                                Resolution.rawToString(
                                                        camResolutionWidthPx, camResolutionHeightPx
                                                )
                                        )
                                )
                        ),
                camDiagonalFOVDeg,
                avgFPS
        );
    }

    public double getCamDiagonalFOVDeg() {
        return camDiagonalFOVDeg;
    }

    public List<Resolution> getResolutions() {
        return resolutions;
    }

    public Resolution getFirstResolution() {
        return resolutions.get(0);
    }

    public double getAvgFPS() {
        return avgFPS;
    }

    public enum Resolution {
        R1280x800(1280, 800),
        R1280x720(1280, 720),
        R800x600(800, 600),
        R960x720(960, 720),
        R320x240(320, 240),
        R640x480(640, 480);

        private final int width;
        private final int height;

        private static final Map<Integer, Map<Integer, Resolution>> resolutionsMap = Arrays.stream(values())
                .collect(Collectors.toUnmodifiableMap(
                        Resolution::getWidth,
                        resolution -> new HashMap<>(Map.of(resolution.getHeight(), resolution)),
                        (existingMap, nextMap) -> {
                            existingMap.putAll(nextMap);
                            return existingMap;
                        }
                ));

        Resolution(final int width, final int height) {
            this.width = width;
            this.height = height;
        }

        public static Optional<Resolution> getFromWidthHeight(final int width, final int height) {
            final Map<Integer, Resolution> heightMap;
            return (heightMap = resolutionsMap.get(width)) == null
                    ? Optional.empty()
                    : Optional.ofNullable(heightMap.get(height));
        }

        public int getWidth() {
            return width;
        }

        public int getHeight() {
            return height;
        }

        public static String rawToString(final int width, final int height) {
            return String.format("%s(width=%d, height=%d)", Resolution.class, width, height);
        }

        @Override
        public String toString() {
            return String.format("%s(width=%d, height=%d)", super.toString(), width, height);
        }
    }
}