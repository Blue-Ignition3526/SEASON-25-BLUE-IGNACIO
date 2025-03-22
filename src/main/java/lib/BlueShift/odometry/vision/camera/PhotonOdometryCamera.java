// ! This file reports some errors but compiles successfully 🙃
package lib.BlueShift.odometry.vision.camera;

import java.util.Optional;
import java.util.function.Function;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import lib.BlueShift.odometry.vision.OdometryCamera;
import lib.BlueShift.odometry.vision.VisionOdometryPoseEstimate;

public class PhotonOdometryCamera implements OdometryCamera {
    private final PhotonPoseEstimator m_poseEstimator;
    private final PhotonCamera m_camera;
    private final Function<VisionOdometryPoseEstimate, Matrix<N3, N1>> m_stdDevProvider;
    // private final Transform3d m_robotToCamera;
    // private Pose2d m_last_pose;
    private boolean m_enabled;
    private double lastTimestamp = -1;

    public PhotonOdometryCamera(PhotonCamera camera, Transform3d robotToCamera, boolean enabled, Function<VisionOdometryPoseEstimate, Matrix<N3, N1>> stdDevProvider) {
        this.m_camera = camera;
        
        // this.m_robotToCamera = robotToCamera;
        this.m_enabled = enabled;
        this.m_poseEstimator = new PhotonPoseEstimator(AprilTagFields.kDefaultField.loadAprilTagLayoutField(), PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, robotToCamera);
        this.m_poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.AVERAGE_BEST_TARGETS);
        this.m_stdDevProvider = stdDevProvider;
    }

    @Override
    public String getCameraName() {
        return m_camera.getName();
    }

    @Override
    public synchronized void disable() {
        m_enabled = false;
    }

    @Override
    public synchronized void enable() {
        m_enabled = true;
    }

    @Override
    public synchronized void setEnabled(boolean enabled) {
        m_enabled = enabled;
    }

    @Override
    public synchronized boolean isEnabled() {
        return m_enabled;
    }

    @Override
    public synchronized double getLastTimestamp() {
        return lastTimestamp;
    }

    @Override
    public synchronized Optional<VisionOdometryPoseEstimate> getEstimate() {
        if (!m_camera.isConnected()) return Optional.empty();
        if (!m_enabled) return Optional.empty();
        // m_poseEstimator.setReferencePose(m_last_pose);
        Optional<EstimatedRobotPose> estimatedPose = m_poseEstimator.update(this.m_camera.getAllUnreadResults().get(0));
        if (estimatedPose.isEmpty()) return Optional.empty();
        double avgTargetDist = 0;
        for (int i = 0; i < estimatedPose.get().targetsUsed.size(); i++) {
            Transform3d transform = estimatedPose.get().targetsUsed.get(i).getBestCameraToTarget();
            avgTargetDist += Math.sqrt(transform.getTranslation().getX() * transform.getTranslation().getX() + transform.getTranslation().getY() * transform.getTranslation().getY() + transform.getTranslation().getZ() * transform.getTranslation().getZ());
        }
        avgTargetDist /= estimatedPose.get().targetsUsed.size();
        lastTimestamp = estimatedPose.get().timestampSeconds;
        // m_last_pose = estimatedPose.get().estimatedPose.toPose2d();
        VisionOdometryPoseEstimate result = new VisionOdometryPoseEstimate(
            estimatedPose.get().estimatedPose.toPose2d(),
            estimatedPose.get().timestampSeconds,
            estimatedPose.get().targetsUsed.size(),
            avgTargetDist
        );
        result.setStdDev(m_stdDevProvider.apply(result));
        return Optional.of(result);
    }
}
