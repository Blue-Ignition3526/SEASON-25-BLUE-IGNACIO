package lib.BlueShift.odometry.vision.camera;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import lib.BlueShift.odometry.vision.VisionOdometryPoseEstimate;

public class VisionOdometryFilters {
    public static Matrix<N3, N1> noFilter(VisionOdometryPoseEstimate estimate) {
        return VecBuilder.fill(0, 0, 0);
    }

    public static Matrix<N3, N1> limelightFilter(VisionOdometryPoseEstimate estimate) {
        return VecBuilder.fill(0.7, 0.7, 999999999d);
    }
}
