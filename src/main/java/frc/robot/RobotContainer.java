package frc.robot;

import java.util.HashMap;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.DriveFeedforwards;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.ArmPivotConstants;
import frc.robot.commands.DriveSwerve;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.CoralIntakeRollers;
import frc.robot.subsystems.CoralIntakeArm;
import frc.robot.subsystems.AlgaeClimbertakePivot;
import frc.robot.subsystems.AlgaeClimbertakeRollers;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.SwerveModule;
import frc.robot.subsystems.CoralIntakeArm.ArmPosition;
import frc.robot.subsystems.CoralIntakeWrist.WristPosition;
import frc.robot.subsystems.CoralIntakeWrist;
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

// TODO: PID DISABLED BEHAVIORS IN SUBSYSTEMS ( setvoltage disabled PID, setpoint enables pid if it has an encoder )
public class RobotContainer {
  // * Controllers
  private final int m_driverControllerPort = 0;
  private final CustomController m_driverControllerCustom = new CustomController(m_driverControllerPort, CustomControllerType.XBOX);

  // * Swerve Drive
  // Swerve modules
  private final SwerveModule frontLeft = new SwerveModule(Constants.SwerveDriveConstants.SwerveModuleConstants.kFrontLeftOptions);
  private final SwerveModule frontRight = new SwerveModule(Constants.SwerveDriveConstants.SwerveModuleConstants.kFrontRightOptions);
  private final SwerveModule backLeft = new SwerveModule(Constants.SwerveDriveConstants.SwerveModuleConstants.kBackLeftOptions);
  private final SwerveModule backRight = new SwerveModule(Constants.SwerveDriveConstants.SwerveModuleConstants.kBackRightOptions);
  
  // Gyro
  private final Gyro m_gyro;
  
  // Swerve Drive
  private final SwerveDrive m_swerveDrive;

  // Speed alterators
  private final SpeedAlterator m_speedAlterator_turn180;
  private final SpeedAlterator m_speedAlterator_lookAt;
  private final SpeedAlterator m_speedAlterator_goTo0;
  private final SpeedAlterator m_speedAlterator_gotTo1;
  private final SpeedAlterator m_speedAlterator_backUp;

  // * Elevator
  private final Elevator m_elevator;

  // * Climbertake
  private final AlgaeClimbertakePivot m_climbertakePivot;
  private final AlgaeClimbertakeRollers m_intakeAlgea;

  // * Coral intake
  private final CoralIntakeWrist m_wrist;
  private final CoralIntakeArm m_armPivot;
  private final CoralIntakeRollers m_intakeCoral;
  
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
    this.m_speedAlterator_backUp = new BackUp(-0.1, m_gyro::getHeading);
    
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

    AutoBuilder.configure(
      m_odometry::getEstimatedPosition,
      m_odometry::resetPosition,
      m_swerveDrive::getRobotRelativeChassisSpeeds,
      (ChassisSpeeds speeds, DriveFeedforwards ff) -> m_swerveDrive.drive(speeds),
      new PPHolonomicDriveController(
        new PIDConstants(0.5),
        new PIDConstants(0.5)
      ),
      ppRobotConfig,
      () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
      m_swerveDrive
    );

    // Build auto chooser
    this.m_autonomousChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("AutoChooser", m_autonomousChooser);
    
    // * Elevator
    this.m_elevator = new Elevator();
    
    // * Climbertake
    m_climbertakePivot = new AlgaeClimbertakePivot();
    m_intakeAlgea = new AlgaeClimbertakeRollers();
    
    // * Coral intake
    m_wrist = new CoralIntakeWrist();
    m_armPivot = new CoralIntakeArm();
    m_intakeCoral = new CoralIntakeRollers();
    
    // * Dashboard testing commands
    // Chassis
    SmartDashboard.putData("SwerveDrive/ResetTurningEncoders", new InstantCommand(m_swerveDrive::resetTurningEncoders).ignoringDisable(true));

    // Elevator
    SmartDashboard.putData("Elevator/50", m_elevator.setSetpointCommand(50).ignoringDisable(true));
    SmartDashboard.putData("Elevator/10", m_elevator.setSetpointCommand(10).ignoringDisable(true));
    SmartDashboard.putData("Elevator/0", m_elevator.setSetpointCommand(0).ignoringDisable(true));

    // Climbertake pivot
    SmartDashboard.putData("Climbertake/Pivot/IntakeAngleCommand", m_climbertakePivot.setSetpointCommand(Constants.ClimbertakeConstants.Pivot.kIntakeAngle).ignoringDisable(true));
    SmartDashboard.putData("Climbertake/Pivot/StoreAngleCommand", m_climbertakePivot.setSetpointCommand(Constants.ClimbertakeConstants.Pivot.kStoreAngle).ignoringDisable(true));
    SmartDashboard.putData("Climbertake/Pivot/ClimbHighAngleCommand", m_climbertakePivot.setSetpointCommand(Constants.ClimbertakeConstants.Pivot.kClimbHighAngle).ignoringDisable(true));
    SmartDashboard.putData("Climbertake/Pivot/ClimbLowAngleCommand", m_climbertakePivot.setSetpointCommand(Constants.ClimbertakeConstants.Pivot.kClimbLowAngle).ignoringDisable(true));

    // Wrist
    SmartDashboard.putData("Wrist/Perpendicular", m_wrist.setSetpointCommand(WristPosition.PERPENDICULAR).ignoringDisable(true));
    SmartDashboard.putData("Wrist/Parallel", m_wrist.setSetpointCommand(WristPosition.PARALLEL).ignoringDisable(true));

    // Arm pivot
    SmartDashboard.putData("ArmPivot/ResetAngle", m_armPivot.resetAngleCommand().ignoringDisable(true));
    SmartDashboard.putData("ArmPivot/LowAngle", m_armPivot.setSetpointCommand(ArmPivotConstants.kLowAngle).ignoringDisable(true));
    SmartDashboard.putData("ArmPivot/MidAngle", m_armPivot.setSetpointCommand(ArmPivotConstants.kMidAngle).ignoringDisable(true));
    SmartDashboard.putData("ArmPivot/HighAngle", m_armPivot.setSetpointCommand(ArmPivotConstants.kHighAngle).ignoringDisable(true));

    // * Add controller bindings
    configureBindings();
  }

  private void configureBindings() {
    // * Swerve Drive
    this.m_swerveDrive.setDefaultCommand(new DriveSwerve(
        m_swerveDrive,
        () -> -m_driverControllerCustom.getLeftY(),
        () -> -m_driverControllerCustom.getLeftX(),
        () -> -m_driverControllerCustom.getRightX(),
        () -> true
      )
    );

    this.m_driverControllerCustom.rightButton().onTrue(this.m_swerveDrive.zeroHeadingCommand());

    // * Elevator
    m_driverControllerCustom.povUp().whileTrue(m_elevator.setVoltageCommand(8));
    m_driverControllerCustom.povUp().onFalse(m_elevator.stopCommand());
    
    m_driverControllerCustom.povDown().whileTrue(m_elevator.setVoltageCommand(-8));
    m_driverControllerCustom.povDown().onFalse(m_elevator.stopCommand());

    // * Climbertake pivot
    this.m_driverControllerCustom.rightBumper().onTrue(this.m_climbertakePivot.setVoltageCommand(-8));
    this.m_driverControllerCustom.rightBumper().onFalse(this.m_climbertakePivot.setVoltageCommand(0));
 
    this.m_driverControllerCustom.leftBumper().onTrue(this.m_climbertakePivot.setVoltageCommand(8));
    this.m_driverControllerCustom.leftBumper().onFalse(this.m_climbertakePivot.setVoltageCommand(0));

    // * Climbertake
    this.m_driverControllerCustom.leftButton().onTrue(this.m_intakeAlgea.setInCommand());
    this.m_driverControllerCustom.topButton().onTrue(this.m_intakeAlgea.setOutCommand());

    this.m_driverControllerCustom.leftButton().onFalse(this.m_intakeAlgea.stopCommand());
    this.m_driverControllerCustom.topButton().onFalse(this.m_intakeAlgea.stopCommand());

    // * Coral Intake
    this.m_driverControllerCustom.leftButton().onTrue(this.m_intakeCoral.setInCommand());
    this.m_driverControllerCustom.leftButton().onFalse(this.m_intakeCoral.stopCommand());

    this.m_driverControllerCustom.topButton().onTrue(this.m_intakeCoral.setOutCommand());
    this.m_driverControllerCustom.topButton().onFalse(this.m_intakeCoral.stopCommand());

    // * Wrist
    this.m_driverControllerCustom.povLeft().onTrue(this.m_wrist.setSetpointCommand(WristPosition.PARALLEL));
    this.m_driverControllerCustom.povRight().onTrue(this.m_wrist.setSetpointCommand(WristPosition.PERPENDICULAR));

    // * Arm
    this.m_driverControllerCustom.leftTrigger().onTrue(this.m_armPivot.setSetpointCommand(ArmPosition.HIGH));
    this.m_driverControllerCustom.leftTrigger().onTrue(this.m_armPivot.setSetpointCommand(ArmPosition.HORIZONTAL));
  }

  public Command getAutonomousCommand() {
    return m_autonomousChooser.getSelected();
  }
}
