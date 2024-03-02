package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.constants.Constants;
import org.photonvision.PhotonCamera;
import org.photonvision.simulation.SimCameraProperties;

public enum TitanCamera {
    //TODO: put real numbers here -> do not use PERFECT_90DEG
    PHOTON_FL_APRILTAG(
            "FL_Apriltag",
            Constants.Vision.ROBOT_TO_FL_APRILTAG_CAM,
            CameraProperties.SUPER_SHAM,  //TODO: GET FOR NEW CAMERAS
            TitanCameraCalibration.fromSimCameraProperties(SimCameraProperties.LL2_640_480()),
            false
    ),
    //TODO: put real numbers here -> do not use PERFECT_90DEG
    PHOTON_FR_APRILTAG(
            "FR_Apriltag",
            Constants.Vision.ROBOT_TO_FR_APRILTAG_CAM,
            CameraProperties.SUPER_SHAM, //TODO: GET FOR NEW CAMERAS
            TitanCameraCalibration.fromSimCameraProperties(SimCameraProperties.LL2_640_480()),
            false
    );

    private final PhotonCamera photonCamera;
    private final Transform3d robotRelativeToCameraTransform;
    private final CameraProperties cameraProperties;
    private final TitanCameraCalibration cameraCalibration;
    private final boolean driverCam;

    TitanCamera(
            final String photonCameraName,
            final Transform3d robotRelativeToCameraTransform,
            final CameraProperties cameraProperties,
            final TitanCameraCalibration titanCameraCalibration,
            final boolean driverCam
    ) {
        this.photonCamera = new PhotonCamera(photonCameraName);
        this.robotRelativeToCameraTransform = robotRelativeToCameraTransform;
        this.cameraProperties = cameraProperties;
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

    public TitanCameraCalibration getCameraCalibration() {
        return cameraCalibration;
    }

    public boolean isDriverCam() {
        return driverCam;
    }

    public SimCameraProperties toSimCameraProperties(final CameraProperties.Resolution resolution) {
        final SimCameraProperties simCameraProperties = cameraCalibration.getSimCameraProperties(resolution);
        simCameraProperties.setFPS(cameraProperties.getAvgFPS());

        return simCameraProperties;
    }

    public SimCameraProperties toSimCameraProperties() {
        return toSimCameraProperties(cameraProperties.getFirstResolution());
    }
}