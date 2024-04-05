package frc.robot.subsystems.vision;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.constants.Constants;
import org.photonvision.PhotonCamera;
import org.photonvision.simulation.SimCameraProperties;

public enum TitanCamera {
    PHOTON_FL_APRILTAG(
            "FL_Apriltag",
            Constants.Vision.ROBOT_TO_FL_APRILTAG,
            CameraProperties.SEE3CAM_24CUG,
            1.0,
            new TitanCameraCalibration()
                    .withCalibration(
                            CameraProperties.Resolution.R1920x1080,
                            MatBuilder.fill(
                                    Nat.N3(),
                                    Nat.N3(),
                                    // intrinsic
                                    1040.7268360455328,
                                    0.0,
                                    988.2378226542257,
                                    0.0,
                                    1040.4068711002674,
                                    544.230023439286,
                                    0.0,
                                    0.0,
                                    1.0
                            ),
                            VecBuilder.fill( // distort
                                    -0.35148800442491696,
                                    0.16199158054202314,
                                    0.0003847806133909519,
                                    0.000042723769639477994,
                                    -0.042523738490321664
                            )
                    )
                    .withCalibrationError(
                            CameraProperties.Resolution.R1920x1080,
                            0.15223032073535464,
                            0.06
                    )
                    .withFPS(
                            CameraProperties.Resolution.R1920x1080,
                            60
                    )
                    .withLatency(
                            CameraProperties.Resolution.R1920x1080,
                            7,
                            3
                    ),
            false
    ),
    PHOTON_FC_APRILTAG(
            "FC_Apriltag",
            Constants.Vision.ROBOT_TO_FC_APRILTAG,
            CameraProperties.ARDUCAM_OV9281,
            1.5,
            new TitanCameraCalibration()
                    .withCalibration(
                            CameraProperties.Resolution.R640x480,
                            MatBuilder.fill(
                                    Nat.N3(),
                                    Nat.N3(),
                                    // intrinsic
                                    549.0659323788205,
                                    0.0,
                                    321.9593286259019,
                                    0.0,
                                    547.4680535283653,
                                    269.83116533897265,
                                    0.0,
                                    0.0,
                                    1.0
                            ),
                            // TODO: mrcal gives us 8 distCoeffs, PV sim expects 5
                            SimCameraProperties.PERFECT_90DEG().getDistCoeffs()
                    )
                    .withCalibrationError(
                            CameraProperties.Resolution.R640x480,
                            0.54,
                            0.06
                    )
                    .withFPS(
                            CameraProperties.Resolution.R640x480,
                            130
                    )
                    .withLatency(
                            CameraProperties.Resolution.R640x480,
                            6,
                            3
                    ),
            false
    ),
    PHOTON_FR_APRILTAG(
            "FR_Apriltag",
            Constants.Vision.ROBOT_TO_FR_APRILTAG,
            CameraProperties.SEE3CAM_24CUG,
            1.0,
            new TitanCameraCalibration()
                    .withCalibration(
                            CameraProperties.Resolution.R1920x1080,
                            MatBuilder.fill(
                                    Nat.N3(),
                                    Nat.N3(),
                                    // intrinsic
                                    1040.7268360455328,
                                    0.0,
                                    988.2378226542257,
                                    0.0,
                                    1040.4068711002674,
                                    544.230023439286,
                                    0.0,
                                    0.0,
                                    1.0
                            ),
                            VecBuilder.fill( // distort
                                    -0.35148800442491696,
                                    0.16199158054202314,
                                    0.0003847806133909519,
                                    0.000042723769639477994,
                                    -0.042523738490321664
                            )
                    )
                    .withCalibrationError(
                            CameraProperties.Resolution.R1920x1080,
                            0.15223032073535464,
                            0.06
                    )
                    .withFPS(
                            CameraProperties.Resolution.R1920x1080,
                            60
                    )
                    .withLatency(
                            CameraProperties.Resolution.R1920x1080,
                            7,
                            3
                    ),
            false
    );

    private final PhotonCamera photonCamera;
    private final Transform3d robotRelativeToCameraTransform;
    private final CameraProperties cameraProperties;
    private final double stdDevFactor;
    private final TitanCameraCalibration cameraCalibration;
    private final boolean driverCam;

    TitanCamera(
            final String photonCameraName,
            final Transform3d robotRelativeToCameraTransform,
            final CameraProperties cameraProperties,
            final double stdDevFactor,
            final TitanCameraCalibration titanCameraCalibration,
            final boolean driverCam
    ) {
        this.photonCamera = new PhotonCamera(photonCameraName);
        this.robotRelativeToCameraTransform = robotRelativeToCameraTransform;
        this.cameraProperties = cameraProperties;
        this.stdDevFactor = stdDevFactor;
        this.cameraCalibration = titanCameraCalibration;
        this.driverCam = driverCam;

        // if it isn't a driverCam, then it should have proper calibration data
        if (!driverCam) {
            for (final CameraProperties.Resolution resolution : cameraProperties.getResolutions()) {
                if (!cameraCalibration.hasResolution(resolution)) {
                    throw new RuntimeException(
                            String.format(
                                    "Camera %s(%s) does not have calibration data for specified %s",
                                    photonCameraName, cameraProperties, resolution
                            )
                    );
                }
            }
        }

        this.photonCamera.setDriverMode(driverCam);
    }

    TitanCamera(
            final String photonCameraName,
            final Transform3d robotRelativeToCameraTransform,
            final CameraProperties cameraProperties,
            final boolean driverCam
    ) {
        this(
                photonCameraName,
                robotRelativeToCameraTransform,
                cameraProperties,
                1.0,
                TitanCameraCalibration.fromSimCameraProperties(SimCameraProperties.PERFECT_90DEG()),
                driverCam
        );
    }

    public PhotonCamera getPhotonCamera() {
        return photonCamera;
    }

    public Transform3d getRobotRelativeToCameraTransform() {
        return robotRelativeToCameraTransform;
    }

    public CameraProperties getCameraProperties() {
        return cameraProperties;
    }

    public double getStdDevFactor() { return stdDevFactor; }

    public TitanCameraCalibration getCameraCalibration() {
        return cameraCalibration;
    }

    public boolean isDriverCam() {
        return driverCam;
    }

    public SimCameraProperties toSimCameraProperties(final CameraProperties.Resolution resolution) {
        return cameraCalibration.getSimCameraProperties(resolution);
    }

    public SimCameraProperties toSimCameraProperties() {
        return toSimCameraProperties(cameraProperties.getFirstResolution());
    }
}