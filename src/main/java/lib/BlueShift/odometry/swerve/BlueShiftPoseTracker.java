package lib.BlueShift.odometry.swerve;

import java.util.HashMap;
import java.util.Optional;
import java.util.function.Supplier;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import lib.BlueShift.odometry.vision.OdometryCamera;
import lib.BlueShift.odometry.vision.VisionOdometryPoseEstimate;
import lib.BlueShift.odometry.vision.camera.LimelightOdometryCamera;

public class BlueShiftPoseTracker extends SubsystemBase {
    // * Chassis information suppliers
    private final Supplier<SwerveModulePosition[]> modulePositionsSupplier;
    private final Supplier<Rotation2d> headingSupplier;

    // * Frequencies
    private final int visionFrequency;
    private final int stateFrequency;

    // * Pose estimator
    private final SwerveDrivePoseEstimator estimator;

    // * Vision enabled flag
    private boolean visionEnabled;

    // * Field for logging odometry
    private final Field2d field = new Field2d();

    // * Camera entry (Container for camera and thread)
    class CameraEntry {
        public OdometryCamera camera;
        public Thread thread;
        public double lastTime;
    }
    private final HashMap<String, CameraEntry> cameras = new HashMap<>();

    // Timekeeping
    private double lastStateTime = 0;
    private Thread stateThread;

    public BlueShiftPoseTracker(
        SwerveDriveKinematics kinematics,
        Supplier<SwerveModulePosition[]> modulePositionsSupplier,
        Supplier<Rotation2d> headingSupplier,
        Pose2d initialPose,
        int visionFrequency,
        int stateFrequency,
        boolean visionEnabled
    ) {
        // * Store the suppliers
        this.modulePositionsSupplier = modulePositionsSupplier;
        this.headingSupplier = headingSupplier;

        // * Store the frequencies
        this.visionFrequency = visionFrequency;
        this.stateFrequency = stateFrequency;

        // * Create the estimator
        this.estimator = new SwerveDrivePoseEstimator(
            kinematics,
            headingSupplier.get(),
            modulePositionsSupplier.get(),
            initialPose
        );

        // * Store the vision enabled flag
        this.visionEnabled = visionEnabled;

        // * Create the state thread
        stateThread = new Thread(() -> {
            while (true) {
                // Timekeeping
                double currentTime = Timer.getFPGATimestamp();
                double dt = currentTime - lastStateTime;
    
                // Skip if the state frequency has not been reached
                if (dt < 1.0 / this.stateFrequency) return;
                else if (dt > 1.0 / this.stateFrequency) System.out.println("State update is running behind!");
    
                // Update the estimator
                estimator.update(headingSupplier.get(), modulePositionsSupplier.get());
    
                // Timekeeping
                lastStateTime = currentTime;
            }
        }, "BSPT-State-Thread");
        stateThread.setPriority(8);
    }

    public synchronized void setVisionEnabled(boolean enabled) {
        visionEnabled = enabled;
    }

    public synchronized Pose2d getEstimatedPosition() {
        return estimator.getEstimatedPosition();
    }

    public synchronized void resetPosition(Pose2d pose) {
        estimator.resetPosition(headingSupplier.get(), modulePositionsSupplier.get(), pose);
    }

    public synchronized void addVisionPose(Pose2d pose, double timestamp, Matrix<N3, N1> stdDev) {
        estimator.addVisionMeasurement(pose, timestamp, stdDev);
    }

    public synchronized void addCamera(OdometryCamera camera) {
        CameraEntry entry = new CameraEntry();
        Thread thread = new Thread(() -> {
            while (visionEnabled) {
                // Timekeeping
                double currentTime = Timer.getFPGATimestamp();
                double dt = currentTime - entry.lastTime;

                // Skip if the camera is disabled
                if (!camera.isEnabled()) continue;

                // Skip if the vision frequency has not been reached
                if (dt < 1.0 / visionFrequency) continue;
                else if (dt > 1.0 / visionFrequency) System.out.println("Vision thread for " + camera.getCameraName() + " is running behind!");

                // Update the camera's heading if it is a LimelightOdometryCamera
                if (camera instanceof LimelightOdometryCamera) ((LimelightOdometryCamera)camera).setHeading(headingSupplier.get().getDegrees());

                // Get the estimate from the camera and add it to the estimator
                Optional<VisionOdometryPoseEstimate> estimate = camera.getEstimate();
                if (estimate.isEmpty()) continue;
                addVisionPose(estimate.get().pose, estimate.get().timestamp, estimate.get().stdDev);

                // Timekeeping
                entry.lastTime = currentTime;
            }
        }, "BSPT-" + camera.getCameraName() + "-Thread");
        thread.setPriority(6);
        
        entry.camera = camera;
        entry.thread = thread;
        cameras.put(camera.getCameraName(), entry);
    }

    /**
     * Starts the vision threads
     * Automatically enables vision
     */
    public synchronized void startVision() {
        setVisionEnabled(true);
        for (var entry : cameras.values()) {
            entry.lastTime = Timer.getFPGATimestamp();
            entry.thread.start();
        }
    }

    /**
     * Stops the vision threads
     * Automatically disables vision
     */
    public synchronized void stopVision() {
        setVisionEnabled(false);
        for (var entry : cameras.values()) {
            try {
                entry.thread.join();
            } catch (Exception e) {
                e.printStackTrace();
            }
        }
    }

    /**
     * Starts the state thread
     */
    public synchronized void start() {
        stateThread.start();
    }

    /**
     * Stops the state thread
     */
    public synchronized void stop() {
        try {
            stateThread.interrupt();
        } catch (Exception e) {
            e.printStackTrace();
        }
    }
}
