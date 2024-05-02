package frc.robot.subsystems.vision.gtsam;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Twist3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.*;

public class GTSAMInterface {
    private static class CameraInterface {
        private final StructArrayPublisher<TagDetection> tagPub;
        private final DoubleArrayPublisher camIntrinsicsPublisher;
        private final StructPublisher<Transform3d> robotToCamPub;
        private final String name;

        public CameraInterface(String name) {
            this.name = name;
            this.tagPub = NetworkTableInstance.getDefault()
                    .getStructArrayTopic("/gtsam_meme/" + name + "/input/tags", TagDetection.struct)
                    .publish(PubSubOption.sendAll(true), PubSubOption.keepDuplicates(true));
            this.robotToCamPub = NetworkTableInstance.getDefault()
                    .getStructTopic("/gtsam_meme/" + name + "/input/robotTcam", Transform3d.struct)
                    .publish(PubSubOption.sendAll(false), PubSubOption.keepDuplicates(false));
            this.camIntrinsicsPublisher = NetworkTableInstance.getDefault()
                    .getDoubleArrayTopic("/gtsam_meme/" + name + "/input/cam_intrinsics")
                    .publish(PubSubOption.sendAll(false), PubSubOption.keepDuplicates(false));
        }
    }

    private final StructPublisher<Twist3d> odomPub;
    private final StructPublisher<Pose3d> guessPub;
    private final Map<String, CameraInterface> cameras = new HashMap<>();

    public GTSAMInterface(final List<String> cameraNames) {
        odomPub = NetworkTableInstance.getDefault()
                .getStructTopic("/gtsam_meme/input/odom_twist", Twist3d.struct)
                .publish(PubSubOption.sendAll(true), PubSubOption.keepDuplicates(true));
        guessPub = NetworkTableInstance.getDefault()
                .getStructTopic("/gtsam_meme/input/pose_initial_guess", Pose3d.struct)
                .publish(PubSubOption.sendAll(true), PubSubOption.keepDuplicates(true));

        cameraNames.stream().map(CameraInterface::new).forEach(it -> cameras.put(it.name, it));
    }

    /**
     * Update the core camera intrinsic parameters. The localizer will apply these
     * as soon as reasonable, and makes no attempt to latency compensate this.
     *
     * @param camName    The name of the camera
     * @param intrinsics Camera intrinsics in standard OpenCV format. See:
     *                   <a href="https://docs.opencv.org/4.x/d9/d0c/group__calib3d.html">here</a>
     */
    public void setCamIntrinsics(final String camName, final Matrix<N3, N3> intrinsics) {
        final CameraInterface cam = cameras.get(camName);
        if (cam == null) {
            throw new RuntimeException("Camera " + camName + " not in map!");
        }

        cam.camIntrinsicsPublisher.set(new double[] {
                intrinsics.get(0, 0),
                intrinsics.get(1, 1),
                intrinsics.get(0, 2),
                intrinsics.get(1, 2),
        });
    }

    /**
     * Update the localizer with new info from this robot loop iteration.
     *
     * @param odomTime The time that the odometry twist from last iteration
     *                 was collected at, in microseconds. WPIUtilJNI::now is
     *                 what I used
     * @param odom     The twist encoding chassis movement from last
     *                 timestamp to now. I use
     *                 SwerveDriveKinematics::toTwist2d
     * @param guess    An (optional, possibly null) initial guess at robot
     *                 pose from solvePNP or prior knowledge.
     */
    public void sendOdomUpdate(final long odomTime, final Twist3d odom, final Pose3d guess) {
        odomPub.set(odom, odomTime);

        if (guess != null) {
            guessPub.set(guess, odomTime);
        }
    }

    /**
     * Update the localizer with new info from this robot loop iteration.
     *
     * @param camName         The name of the camera
     * @param tagDetTime      The time that the frame encoding detected tags
     *                        collected at, in microseconds. WPIUtilJNI::now is what
     *                        I used
     * @param camDetectedTags The list of tags this camera could see
     * @param robotToCam      Transform from robot to camera optical focal point.
     *                        This is not latency compensated yet, so maybe don't
     *                        put your camera on a turret
     */
    public void sendVisionUpdate(final String camName, final long tagDetTime, final List<TagDetection> camDetectedTags,
                                 final Transform3d robotToCam) {
        final CameraInterface cam = cameras.get(camName);
        if (cam == null) {
            throw new RuntimeException("Camera " + camName + " not in map!");
        }

        cam.tagPub.set(camDetectedTags.toArray(new TagDetection[0]), tagDetTime);
        cam.robotToCamPub.set(robotToCam, tagDetTime);
    }
}