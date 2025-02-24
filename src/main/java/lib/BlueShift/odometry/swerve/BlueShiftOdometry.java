package lib.BlueShift.odometry.swerve;

import java.util.Optional;
import java.util.function.Supplier;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import lib.BlueShift.odometry.vision.OdometryCamera;
import lib.BlueShift.odometry.vision.VisionOdometryPoseEstimate;
import lib.BlueShift.odometry.vision.camera.LimelightOdometryCamera;

public class BlueShiftOdometry extends SubsystemBase {
    // Pose estimator
    private final SwerveDrivePoseEstimator m_poseEstimator;

    // Suppliers
    private final Supplier<Rotation2d> m_gyroAngleSupplier;
    private final Supplier<SwerveModulePosition[]> m_modulePositionsSupplier;
    
    // Cameras
    private final OdometryCamera[] m_cameras;
    
    // Notifiers
    private final Notifier m_visionNotifier;
    private final double m_visionPeriod;

    // Field 
    private final Field2d m_field = new Field2d();

    public BlueShiftOdometry(
        SwerveDriveKinematics kinematics,
        Supplier<Rotation2d> gyroAngleSupplier,
        Supplier<SwerveModulePosition[]> modulePositionsSupplier,
        Pose2d initialPose,
        double visionPeriod,
        OdometryCamera... cameras
    ) {
        // Pose estimator
        this.m_poseEstimator = new SwerveDrivePoseEstimator(
            kinematics,
            gyroAngleSupplier.get(),
            modulePositionsSupplier.get(),
            initialPose
        );

        // Suppliers
        this.m_gyroAngleSupplier = gyroAngleSupplier;
        this.m_modulePositionsSupplier = modulePositionsSupplier;

        // Cameras
        this.m_cameras = cameras;

        // Notifiers
        this.m_visionNotifier = new Notifier(this::updateVision);
        this.m_visionPeriod = visionPeriod;
    }

    /**
     * Start vision processing
     */
    public synchronized void startVision() {
        m_visionNotifier.startPeriodic(m_visionPeriod);
    }

    /**
     * Stop vision processing
     */
    public synchronized void stopVision() {
        m_visionNotifier.stop();
    }

    /**
     * Update vision odometry
     */
    private synchronized void updateVision() {
        for (var camera : m_cameras) {
            if (!camera.isEnabled()) continue;
            // Set the heading of the robot if it is a LimelightOdometryCamera
            if (camera instanceof LimelightOdometryCamera) ((LimelightOdometryCamera)camera).setHeading(m_gyroAngleSupplier.get().getDegrees());
            Optional<VisionOdometryPoseEstimate> estimate = camera.getEstimate();
            if (estimate.isEmpty()) continue;
            m_poseEstimator.addVisionMeasurement(estimate.get().pose, estimate.get().timestamp, estimate.get().stdDev);
        }
    }

    /**
     * Resets the positon to the given pose
     * @param pose
     */
    public synchronized void resetPosition(Pose2d pose) {
        this.m_poseEstimator.resetPosition(this.m_gyroAngleSupplier.get(), m_modulePositionsSupplier.get(), pose);
    }

    /**
     * Get the estimated position
     * @return
     */
    public synchronized Pose2d getEstimatedPosition() {
        return m_poseEstimator.getEstimatedPosition();
    }

    /**
     * Set the pose to the current pose returned by the first camera
     */
    public synchronized void setVisionPose() {
        Optional<VisionOdometryPoseEstimate> estimate = m_cameras[0].getEstimate();
        if (estimate.isEmpty()) return;
        m_poseEstimator.resetPosition(this.m_gyroAngleSupplier.get(), this.m_modulePositionsSupplier.get(), estimate.get().pose);
    }

    @Override
    public void periodic() {
        // Update state odometry
        m_poseEstimator.update(
            m_gyroAngleSupplier.get(),
            m_modulePositionsSupplier.get()
        );

        // Update field pose
        m_field.setRobotPose(m_poseEstimator.getEstimatedPosition());

        // Send field to dash
        SmartDashboard.putData("Field", m_field);
    }
}
