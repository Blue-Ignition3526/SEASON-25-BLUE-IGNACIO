package lib.BlueShift.odometry;

import edu.wpi.first.math.geometry.Pose2d;

public interface EstimatedPositionProvider {
    public Pose2d getEstimatedPosition();
}
