package frc.robot;

import java.util.HashMap;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.DriveSwerve;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.SwerveModule;
import frc.robot.subsystems.Gyro.Gyro;
import frc.robot.subsystems.Gyro.GyroIOPigeon;
import lib.Elastic;
import lib.Elastic.ElasticNotification;
import lib.Elastic.ElasticNotification.NotificationLevel;
import lib.BlueShift.control.CustomController;
import lib.BlueShift.control.CustomController.CustomControllerType;
import lib.BlueShift.odometry.swerve.BlueShiftOdometry;
import lib.BlueShift.odometry.vision.camera.LimelightOdometryCamera;
import lib.BlueShift.odometry.vision.camera.VisionOdometryFilters;
import lib.BlueShift.control.SpeedAlterator;
import frc.robot.speedAlterators.*;

public class RobotContainer {
  // * Controllers
  private final int m_driverControllerPort = 0;
  private final CustomController m_driverControllerCustom = new CustomController(m_driverControllerPort, CustomControllerType.XBOX);
  
  // * Swerve Modules
  private final SwerveModule frontLeft = new SwerveModule(Constants.SwerveDriveConstants.SwerveModuleConstants.kFrontLeftOptions);
  private final SwerveModule frontRight = new SwerveModule(Constants.SwerveDriveConstants.SwerveModuleConstants.kFrontRightOptions);
  private final SwerveModule backLeft = new SwerveModule(Constants.SwerveDriveConstants.SwerveModuleConstants.kBackLeftOptions);
  private final SwerveModule backRight = new SwerveModule(Constants.SwerveDriveConstants.SwerveModuleConstants.kBackRightOptions);

  // * Swerve Drive
  private final SwerveDrive m_swerveDrive;

  // * Gyro
  private final Gyro m_gyro;

  // * Speed alterators
  private final SpeedAlterator m_speedAlterator_turn180;
  private final SpeedAlterator m_speedAlterator_lookAt;
  private final SpeedAlterator m_speedAlterator_goTo0;
  private final SpeedAlterator m_speedAlterator_gotTo1;

  // * Odometry and Vision
  private final LimelightOdometryCamera m_limelight3G;
  private final BlueShiftOdometry m_odometry;
  private final double m_visionPeriod = 0.1;

  // * Autonomous
  private final SendableChooser<Command> m_autonomousChooser;

  public RobotContainer() {
    // * Gyro
    m_gyro = new Gyro(new GyroIOPigeon(Constants.SwerveDriveConstants.kGyroDevice));

    // * Swerve Drive
    m_swerveDrive = new SwerveDrive(frontLeft, frontRight, backLeft, backRight, m_gyro);

    // * Odometry and Vision
    this.m_limelight3G = new LimelightOdometryCamera(Constants.Vision.Limelight3G.kName, false, VisionOdometryFilters::visionFilter);
    this.m_odometry = new BlueShiftOdometry(
      Constants.SwerveDriveConstants.PhysicalModel.kDriveKinematics, 
      m_gyro::getHeading,
      m_swerveDrive::getModulePositions,
      new Pose2d(),
      m_visionPeriod,
      m_limelight3G
    );
    this.m_limelight3G.enable();
    this.m_odometry.startVision();

    // * Speed alterators
    this.m_speedAlterator_turn180 = new Turn180(m_odometry::getEstimatedPosition);
    this.m_speedAlterator_lookAt = new LookController(this.m_gyro::getYaw, this.m_driverControllerCustom::getRightX, this.m_driverControllerCustom::getRightY, 0.1);
    this.m_speedAlterator_goTo0 = new GoToPose(m_odometry::getEstimatedPosition, new Pose2d(0, 0, new Rotation2d()));
    this.m_speedAlterator_gotTo1 = new GoToPose(m_odometry::getEstimatedPosition, new Pose2d(0, 3, Rotation2d.fromDegrees(180)));
    
    // * Autonomous
    // Register commands
    NamedCommands.registerCommands(new HashMap<String, Command>(){{
      put("LeaveCoral-L2", null);
    }});

    // Robot config
    RobotConfig ppRobotConfig = null;
    try{
      ppRobotConfig = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      Elastic.sendAlert(new ElasticNotification(NotificationLevel.ERROR, "ERROR! COULD NOT LOAD PP ROBOT CONFIG", e.getMessage()));
      DriverStation.reportError("ERROR! COULD NOT LOAD PP ROBOT CONFIG", e.getStackTrace());
    }

    // Auto builder
    AutoBuilder.configure(
      m_odometry::getEstimatedPosition, 
      m_odometry::resetPosition,
      m_swerveDrive::getRobotRelativeChassisSpeeds,
      (speeds, ff) -> m_swerveDrive.driveRobotRelative(speeds), 
      new PPHolonomicDriveController(
        new PIDConstants(5.0, 0.0, 0.0),
        new PIDConstants(5.0, 0.0, 0.0)
      ),
      ppRobotConfig,
      () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
      m_swerveDrive
    );

    // Auto chooser
    this.m_autonomousChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto chooser", m_autonomousChooser);

    // * Dashboard commands
    SmartDashboard.putData("SwerveDrive/Zero heading", m_swerveDrive.zeroHeadingCommand().ignoringDisable(true));
    SmartDashboard.putData("SwerveDrive/Reset pose", new InstantCommand(m_odometry::setVisionPose).ignoringDisable(true));

    // * Add controller bindings
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

    this.m_driverControllerCustom.rightBumper().onTrue(m_swerveDrive.enableSpeedAlteratorCommand(m_speedAlterator_turn180));
    this.m_driverControllerCustom.leftBumper().onTrue(m_swerveDrive.enableSpeedAlteratorCommand(m_speedAlterator_lookAt));
    this.m_driverControllerCustom.bottomButton().onTrue(m_swerveDrive.enableSpeedAlteratorCommand(m_speedAlterator_goTo0));
    this.m_driverControllerCustom.topButton().onTrue(m_swerveDrive.enableSpeedAlteratorCommand(m_speedAlterator_gotTo1));
    this.m_driverControllerCustom.rightBumper().onFalse(m_swerveDrive.disableSpeedAlteratorCommand());
    this.m_driverControllerCustom.leftBumper().onFalse(m_swerveDrive.disableSpeedAlteratorCommand());
    this.m_driverControllerCustom.bottomButton().onFalse(m_swerveDrive.disableSpeedAlteratorCommand());
    this.m_driverControllerCustom.topButton().onFalse(m_swerveDrive.disableSpeedAlteratorCommand());
  }

  public Command getAutonomousCommand() {
    return m_autonomousChooser.getSelected();
  }
}
