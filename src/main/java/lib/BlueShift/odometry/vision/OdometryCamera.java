package lib.BlueShift.odometry.vision;

import java.util.Optional;

public interface OdometryCamera {
    public String getCameraName();

    public boolean isEnabled();
    public void setEnabled(boolean enabled);
    public void enable();
    public void disable();
    public Optional<VisionOdometryPoseEstimate> getEstimate();

    /**
     * Returns -1 when invalid
     * @return
     */
    public double getLastTimestamp();
}