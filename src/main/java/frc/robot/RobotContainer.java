package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.DriveSwerve;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.SwerveModule;
import frc.robot.subsystems.Gyro.Gyro;
import frc.robot.subsystems.Gyro.GyroIOPigeon;
import lib.BlueShift.control.CustomController;
import lib.BlueShift.control.CustomController.CustomControllerType;
import lib.BlueShift.odometry.swerve.BlueShiftPoseTracker;
import lib.BlueShift.odometry.swerve.FusedSwerveDrivePoseEstimator;
import lib.BlueShift.odometry.vision.OdometryCamera;
import lib.BlueShift.odometry.vision.camera.LimelightOdometryCamera;
import lib.BlueShift.odometry.vision.camera.VisionOdometryFilters;
import lib.BlueShift.control.SpeedAlterator;
import frc.robot.speedAlterators.*;

public class RobotContainer {
  private final int m_driverControllerPort = 0;
  private final CustomController m_driverControllerCustom = new CustomController(m_driverControllerPort, CustomControllerType.XBOX);
  
  private final SwerveModule frontLeft = new SwerveModule(Constants.SwerveDriveConstants.SwerveModuleConstants.kFrontLeftOptions);
  private final SwerveModule frontRight = new SwerveModule(Constants.SwerveDriveConstants.SwerveModuleConstants.kFrontRightOptions);
  private final SwerveModule backLeft = new SwerveModule(Constants.SwerveDriveConstants.SwerveModuleConstants.kBackLeftOptions);
  private final SwerveModule backRight = new SwerveModule(Constants.SwerveDriveConstants.SwerveModuleConstants.kBackRightOptions);

  private final SwerveDrive m_swerveDrive;
  private final Gyro gyro;

  private final SendableChooser<Command> autonomousChooser;

  private final SpeedAlterator turn180;
  private final SpeedAlterator lookAt;
  private final SpeedAlterator goTo0;
  private final SpeedAlterator gotTo1;

  // Odometry and Vision
  private final LimelightOdometryCamera m_limelight3G;
  private final BlueShiftPoseTracker m_poseTracker;

  public RobotContainer() {
    gyro = new Gyro(new GyroIOPigeon(Constants.SwerveDriveConstants.kGyroDevice));
    m_swerveDrive = new SwerveDrive(frontLeft, frontRight, backLeft, backRight, gyro);

    this.turn180 = new Turn180(this.m_swerveDrive);
    this.lookAt = new LookController(this.gyro::getYaw, this.m_driverControllerCustom::getRightX, this.m_driverControllerCustom::getRightY, 0.1);
    this.goTo0 = new GoToPose(this.m_swerveDrive::getPose, new Pose2d(0, 0, new Rotation2d()));
    this.gotTo1 = new GoToPose(this.m_swerveDrive::getPose, new Pose2d(0, 3, Rotation2d.fromDegrees(180)));

    autonomousChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("ZeroHeading", m_swerveDrive.zeroHeadingCommand());
    SmartDashboard.putData("Autonomous", autonomousChooser);
    SmartDashboard.putData("Reset Odometry", m_swerveDrive.resetPoseCommand());

    // Odometry and Vision
    // ! Vision odometry starts disabled by default
    this.m_limelight3G = new LimelightOdometryCamera(Constants.Vision.Limelight3G.kName, false, VisionOdometryFilters::visionFilter);
    this.m_poseTracker = new BlueShiftPoseTracker(
      Constants.SwerveDriveConstants.PhysicalModel.kDriveKinematics,
      m_swerveDrive::getModulePositions,
      m_swerveDrive::getHeading,
      new Pose2d(),
      60,
      100,
      false
    );
    this.m_poseTracker.addCamera(m_limelight3G);

    // Enable vision measurements and pose estimation
    this.m_limelight3G.enable();
    this.m_poseTracker.start();
    this.m_poseTracker.startVision();

    configureBindings();
  }

  private void configureBindings() {
    this.m_swerveDrive.setDefaultCommand(new DriveSwerve(
        m_swerveDrive,
        () -> -this.m_driverControllerCustom.getLeftY(),
        () -> -this.m_driverControllerCustom.getLeftX(),
        () -> -this.m_driverControllerCustom.getRightX(),
        () -> true
      )
    );

    this.m_driverControllerCustom.rightButton().onTrue(this.m_swerveDrive.zeroHeadingCommand());
    this.m_driverControllerCustom.leftButton().onTrue(this.m_swerveDrive.resetPoseCommand());

    this.m_driverControllerCustom.rightBumper().onTrue(m_swerveDrive.enableSpeedAlteratorCommand(turn180));
    this.m_driverControllerCustom.leftBumper().onTrue(m_swerveDrive.enableSpeedAlteratorCommand(lookAt));
    this.m_driverControllerCustom.bottomButton().onTrue(m_swerveDrive.enableSpeedAlteratorCommand(goTo0));
    this.m_driverControllerCustom.topButton().onTrue(m_swerveDrive.enableSpeedAlteratorCommand(gotTo1));
    this.m_driverControllerCustom.rightBumper().onFalse(m_swerveDrive.disableSpeedAlteratorCommand());
    this.m_driverControllerCustom.leftBumper().onFalse(m_swerveDrive.disableSpeedAlteratorCommand());
    this.m_driverControllerCustom.bottomButton().onFalse(m_swerveDrive.disableSpeedAlteratorCommand());
    this.m_driverControllerCustom.topButton().onFalse(m_swerveDrive.disableSpeedAlteratorCommand());
  }

  public Command getAutonomousCommand() {
    return autonomousChooser.getSelected();
  }
}
