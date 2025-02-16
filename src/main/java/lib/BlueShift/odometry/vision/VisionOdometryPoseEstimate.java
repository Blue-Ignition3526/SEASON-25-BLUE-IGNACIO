package lib.BlueShift.odometry.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public class VisionOdometryPoseEstimate {
    public Pose2d pose;
    public double timestamp;
    public int tagCount;
    public double tagDist;
    public Matrix<N3, N1> stdDev;

    public VisionOdometryPoseEstimate(Pose2d pose, double timestamp, int tagCount, double tagDist) {
        this.pose = pose;
        this.timestamp = timestamp;
        this.tagCount = tagCount;
        this.tagDist = tagDist;
    }

    public void setStdDev(Matrix<N3, N1> stdDev) {
        this.stdDev = stdDev;
    }
}
