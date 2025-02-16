package lib.BlueShift.odometry.vision.camera;

import java.util.Optional;
import java.util.function.Function;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.LimelightHelpers;
import lib.BlueShift.odometry.vision.OdometryCamera;
import lib.BlueShift.odometry.vision.VisionOdometryPoseEstimate;

/**
 * Limelight implentation of OdometryCamera
 * This implementation uses MetaTag2 to get the robot's pose
 * **Make sure to update the heading periodically using the robot's gyro**
 * `LimelightOdometryCamera.setHeading(double degrees)`
 */
public class LimelightOdometryCamera implements OdometryCamera {
    private final String m_cameraName;
    private final Function<VisionOdometryPoseEstimate, Matrix<N3, N1>> m_stdDevProvider;
    private boolean m_enabled;
    private double lastLatency = -1;

    public LimelightOdometryCamera(String cameraName, boolean enabled, Function<VisionOdometryPoseEstimate, Matrix<N3, N1>> stdDevProvider) {
        this.m_cameraName = cameraName;
        this.m_enabled = enabled;
        this.m_stdDevProvider = stdDevProvider;
    }

    public synchronized void setHeading(double degrees) {
        LimelightHelpers.SetRobotOrientation(m_cameraName, degrees, 0, 0, 0, 0, 0);
    }

    @Override
    public String getCameraName() {
        return m_cameraName;
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
    public synchronized Optional<VisionOdometryPoseEstimate> getEstimate() {
        if (!m_enabled) return Optional.empty();
        // LimelightHelpers.PoseEstimate poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(m_cameraName);
        LimelightHelpers.PoseEstimate poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(m_cameraName);
        if (poseEstimate == null || poseEstimate.tagCount < 1) return Optional.empty();
        this.lastLatency = poseEstimate.latency;
        VisionOdometryPoseEstimate result = new VisionOdometryPoseEstimate(
            poseEstimate.pose,
            poseEstimate.timestampSeconds,
            poseEstimate.tagCount,
            poseEstimate.avgTagDist
        );
        result.setStdDev(m_stdDevProvider.apply(result));
        return Optional.of(result);
    }

    @Override
    public synchronized double getLastTimestamp() {
        return lastLatency;
    }

    @Override
    public synchronized boolean isEnabled() {
        return m_enabled;
    }
}
