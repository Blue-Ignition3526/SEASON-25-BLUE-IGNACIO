package lib.BlueShift.odometry.swerve;

import java.util.Optional;
import java.util.function.Supplier;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import lib.BlueShift.odometry.EstimatedPositionProvider;
import lib.BlueShift.odometry.vision.OdometryCamera;
import lib.BlueShift.odometry.vision.VisionOdometryPoseEstimate;
import lib.BlueShift.odometry.vision.camera.LimelightOdometryCamera;

public class FusedSwerveDrivePoseEstimator extends SubsystemBase implements EstimatedPositionProvider {
  private final Supplier<SwerveModulePosition[]> modulePositionsSupplier;
  private final Supplier<Rotation2d> headingSupplier;
  private boolean visionEnabled;

  private final OdometryCamera[] cameras;
  private final SwerveDrivePoseEstimator estimator;

  private final Field2d field = new Field2d();

  public FusedSwerveDrivePoseEstimator(
    SwerveDriveKinematics kinematics,
    Supplier<SwerveModulePosition[]> modulePositionsSupplier,
    Supplier<Rotation2d> headingSupplier,
    OdometryCamera[] cameras,
    Pose2d initialPose,
    boolean visionEnabled
  ) {
    this.modulePositionsSupplier = modulePositionsSupplier;
    this.headingSupplier = headingSupplier;
    this.visionEnabled = visionEnabled;

    this.cameras = cameras;
    this.estimator = new SwerveDrivePoseEstimator(
      kinematics,
      headingSupplier.get(),
      modulePositionsSupplier.get(),
      initialPose
    );
  }

  public synchronized void setVisionEnabled(boolean enabled) {
    visionEnabled = enabled;
  }

  public synchronized void enableVision() {
    visionEnabled = true;
  }

  public synchronized void disableVision() {
    visionEnabled = false;
  }

  public synchronized Pose2d getEstimatedPosition() {
    return estimator.getEstimatedPosition();
  }

  public synchronized Rotation2d getEstimatedHeading() {
    return estimator.getEstimatedPosition().getRotation();
  }

  public synchronized void resetPosition(Pose2d pose) {
    estimator.resetPosition(headingSupplier.get(), modulePositionsSupplier.get(), pose);
  }

  public synchronized void setvisionPose() {
    // Pose2d pose = new Pose2d();
    // int camerasWithPose = 0;
    // for (int i = 0; i < cameras.length; i++) {
    //   OdometryCamera camera = cameras[i];
    //   if (!camera.isEnabled()) continue;
    //   // Set the heading of the robot if it is a LimelightOdometryCamera
    //   if (camera instanceof LimelightOdometryCamera) ((LimelightOdometryCamera)camera).setHeading(this.getEstimatedHeading().getDegrees());
    //   Optional<VisionOdometryPoseEstimate> estimate = camera.getEstimate();
    //   if (estimate.isEmpty()) continue;
    //   camerasWithPose++;
    //   pose = pose.plus(new Transform2d(estimate.get().pose.getTranslation(), estimate.get().pose.getRotation()));
    // }
    // pose = pose.div(camerasWithPose);
    // estimator.resetPosition(headingSupplier.get(), modulePositionsSupplier.get(), pose);
    Optional<VisionOdometryPoseEstimate> estimate = cameras[0].getEstimate();
    if (estimate.isEmpty()) return;
    estimator.resetPosition(headingSupplier.get(), modulePositionsSupplier.get(), estimate.get().pose);
  }

  // TODO: Multithreaded
  public synchronized void update() {
    estimator.update(headingSupplier.get(), modulePositionsSupplier.get());
  
    if (!visionEnabled) return;
    for (var camera : cameras) {
      if (!camera.isEnabled()) continue;
      // Set the heading of the robot if it is a LimelightOdometryCamera
      if (camera instanceof LimelightOdometryCamera) ((LimelightOdometryCamera)camera).setHeading(this.getEstimatedHeading().getDegrees());
      Optional<VisionOdometryPoseEstimate> estimate = camera.getEstimate();
      if (estimate.isEmpty()) continue;
      estimator.addVisionMeasurement(estimate.get().pose, estimate.get().timestamp, estimate.get().stdDev);
    }
  } 

  @Override
  public void periodic() {
    update();
    field.setRobotPose(getEstimatedPosition());
    SmartDashboard.putData("Field", field);
  }
}
