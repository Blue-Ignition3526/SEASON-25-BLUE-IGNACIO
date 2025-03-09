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
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ArmPivotConstants;
import frc.robot.Constants.SwerveDriveConstants;
import frc.robot.Constants.RobotState;
import frc.robot.commands.DriveSwerve;
import frc.robot.commands.CompoundCommands.ScoringCommands;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.CoralIntakeRollers;
import frc.robot.subsystems.CoralIntakeArm;
import frc.robot.subsystems.AlgaeClimbertakePivot;
import frc.robot.subsystems.AlgaeClimbertakeRollers;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.SwerveModule;
import frc.robot.subsystems.CoralIntakeArm.ArmPosition;
import frc.robot.subsystems.CoralIntakeWrist.WristPosition;
import frc.robot.subsystems.Elevator.ElevatorPosition;
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

// TODO: Check ALL POSITIONS
// TODO: add grab position
// TODO: Add intake and outtake to paths
// TODO: Check operator bindings
//TODO: Automate leaving game pieces

//TODO: QUE NO BAJE EL ELEVADOR DEL 0
public class RobotContainer {
  // * Controllers
  private final CustomController DRIVER = new CustomController(0, CustomControllerType.PS5);
  private final CustomController OPERATOR = new CustomController(1, CustomControllerType.PS5);

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
  private final SpeedAlterator m_speedAlterator_backUp;

  // * Elevator
  private final Elevator m_elevator;

  // * Climbertake
  private final AlgaeClimbertakePivot m_algaeClimbertakePivot;
  private final AlgaeClimbertakeRollers m_algaeClimbertakeRollers;

  // * Coral intake
  private final CoralIntakeWrist m_coralIntakeWrist;
  private final CoralIntakeArm m_coralIntakeArm;
  private final CoralIntakeRollers m_coralIntakeRollers;
  
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

    // * Elevator
    this.m_elevator = new Elevator();
    
    // * Climbertake
    m_algaeClimbertakePivot = new AlgaeClimbertakePivot();
    m_algaeClimbertakeRollers = new AlgaeClimbertakeRollers();
    
    // * Coral intake
    m_coralIntakeWrist = new CoralIntakeWrist();
    m_coralIntakeArm = new CoralIntakeArm();
    m_coralIntakeRollers = new CoralIntakeRollers();

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
    this.m_speedAlterator_lookAt = new LookController(this.m_gyro::getYaw, this.DRIVER::getRightX, this.DRIVER::getRightY, 0.1);
    this.m_speedAlterator_backUp = new BackUp(-0.1, m_gyro::getHeading);
    
    // * Autonomous
    // Register commands
    NamedCommands.registerCommands(new HashMap<String, Command>(){{
       put("Score-L1", new SequentialCommandGroup(
        ScoringCommands.scorePositionCommand(RobotState.L1, m_elevator, m_coralIntakeArm, m_coralIntakeWrist),
        new WaitCommand(0.5),
        m_coralIntakeRollers.setOutCommand(),
        new WaitCommand(0.5)
       ));

       put("Intake-Coral", new SequentialCommandGroup(
        ScoringCommands.scorePositionCommand(RobotState.SOURCE, m_elevator, m_coralIntakeArm, m_coralIntakeWrist),
        m_coralIntakeRollers.setInCommand(),
        new WaitCommand(2),
        m_coralIntakeRollers.stopCommand()
       ));
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
      () -> false,
      m_swerveDrive
    );

    // Build auto chooser
    this.m_autonomousChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("AutoChooser", m_autonomousChooser);
    
    // * Dashboard testing commands
    // Chassis
    SmartDashboard.putData("SwerveDrive/ResetTurningEncoders", new InstantCommand(m_swerveDrive::resetTurningEncoders).ignoringDisable(true));

    // Elevator
    SmartDashboard.putData("Elevator/HOME", m_elevator.setSetpointCommand(ElevatorPosition.HOME).ignoringDisable(true));
    SmartDashboard.putData("Elevator/L1", m_elevator.setSetpointCommand(ElevatorPosition.L1).ignoringDisable(true));
    SmartDashboard.putData("Elevator/L2", m_elevator.setSetpointCommand(ElevatorPosition.L2).ignoringDisable(true));
    SmartDashboard.putData("Elevator/L3", m_elevator.setSetpointCommand(ElevatorPosition.L3).ignoringDisable(true));
    SmartDashboard.putData("Elevator/L4", m_elevator.setSetpointCommand(ElevatorPosition.L4).ignoringDisable(true));

    // Climbertake pivot
    SmartDashboard.putData("Climbertake/Pivot/IntakeAngleCommand", m_algaeClimbertakePivot.setSetpointCommand(Constants.ClimbertakeConstants.Pivot.kIntakeAngle).ignoringDisable(true));
    SmartDashboard.putData("Climbertake/Pivot/StoreAngleCommand", m_algaeClimbertakePivot.setSetpointCommand(Constants.ClimbertakeConstants.Pivot.kStoreAngle).ignoringDisable(true));
    SmartDashboard.putData("Climbertake/Pivot/ClimbHighAngleCommand", m_algaeClimbertakePivot.setSetpointCommand(Constants.ClimbertakeConstants.Pivot.kClimbHighAngle).ignoringDisable(true));
    SmartDashboard.putData("Climbertake/Pivot/ClimbLowAngleCommand", m_algaeClimbertakePivot.setSetpointCommand(Constants.ClimbertakeConstants.Pivot.kClimbLowAngle).ignoringDisable(true));

    // Wrist
    SmartDashboard.putData("Wrist/Perpendicular", m_coralIntakeWrist.setSetpointCommand(WristPosition.PERPENDICULAR).ignoringDisable(true));
    SmartDashboard.putData("Wrist/Parallel", m_coralIntakeWrist.setSetpointCommand(WristPosition.PARALLEL).ignoringDisable(true));

    // Arm pivot
    SmartDashboard.putData("ArmPivot/ResetAngle", m_coralIntakeArm.resetAngleCommand().ignoringDisable(true));
    SmartDashboard.putData("ArmPivot/LowAngle", m_coralIntakeArm.setSetpointCommand(ArmPivotConstants.kLowAngle).ignoringDisable(true));
    SmartDashboard.putData("ArmPivot/MidAngle", m_coralIntakeArm.setSetpointCommand(ArmPivotConstants.kMidAngle).ignoringDisable(true));
    SmartDashboard.putData("ArmPivot/HighAngle", m_coralIntakeArm.setSetpointCommand(ArmPivotConstants.kHighAngle).ignoringDisable(true));

    SmartDashboard.putData("Elevator/ResetPosition", m_elevator.resetElevatorPositionCommand().ignoringDisable(true));

    // ! BIND PID RESETS
    Trigger enabledTrigger = new Trigger(DriverStation::isEnabled);
    enabledTrigger.onTrue(new SequentialCommandGroup(
      m_elevator.resetPIDCommand(),
      m_coralIntakeWrist.resetPIDCommand(),
      m_coralIntakeArm.resetPIDCommand()
    ));

    // * Add controller bindings
    configureBindings();
  }

  private void configureBindings() {
    // ! DRIVER BINDINGS
    // * Swerve drive binding
    this.m_swerveDrive.setDefaultCommand(new DriveSwerve(
        m_swerveDrive,
        () -> -DRIVER.getLeftY(),
        () -> -DRIVER.getLeftX(),
        () ->  DRIVER.getLeftTrigger() - DRIVER.getRightTrigger(),
        () -> !DRIVER.bottomButton().getAsBoolean()
      )
    );

    // * Look at speed alterator
    Trigger lookAtTrigger = new Trigger(() -> 
      Math.abs(DRIVER.getRightX()) > SwerveDriveConstants.kJoystickDeadband ||
      Math.abs(DRIVER.getRightY()) > SwerveDriveConstants.kJoystickDeadband
    );

    // Binding
    lookAtTrigger.onTrue(m_swerveDrive.enableSpeedAlteratorCommand(m_speedAlterator_lookAt));
    lookAtTrigger.onFalse(m_swerveDrive.disableSpeedAlteratorCommand());

    // * Turn 180
    this.DRIVER.leftBumper().onTrue(m_swerveDrive.enableSpeedAlteratorCommand(m_speedAlterator_turn180));
    this.DRIVER.leftBumper().onFalse(m_swerveDrive.disableSpeedAlteratorCommand());

    // * Reset heading with right stick button
    this.DRIVER.rightStickButton().onTrue(this.m_swerveDrive.zeroHeadingCommand());

    // * Driver Coral intake
    this.DRIVER.leftButton().onTrue(new ParallelCommandGroup(
      ScoringCommands.scorePositionCommand(RobotState.SOURCE, m_elevator, m_coralIntakeArm, m_coralIntakeWrist),
      m_coralIntakeRollers.setInCommand()
    ));
    this.DRIVER.leftButton().onFalse(new ParallelCommandGroup(
      ScoringCommands.scorePositionCommand(RobotState.L1, m_elevator, m_coralIntakeArm, m_coralIntakeWrist),
      m_coralIntakeRollers.stopCommand()
    ));

    // ! OPERATOR BINDINGS
    // * Manuel Elevator
    // Options
    this.OPERATOR.startButton().whileTrue(m_elevator.setVoltageCommand(10));
    this.OPERATOR.startButton().onFalse(m_elevator.stopCommand());

    // Share
    this.OPERATOR.backButton().whileTrue(m_elevator.setVoltageCommand(-10));
    this.OPERATOR.backButton().onFalse(m_elevator.stopCommand());

    // * Manuel Climbertake pivot
    // Ready
    this.OPERATOR.rightBumper().onTrue(this.m_algaeClimbertakePivot.setVoltageCommand(-8));
    this.OPERATOR.rightBumper().onFalse(this.m_algaeClimbertakePivot.setVoltageCommand(0));
 
    this.OPERATOR.leftBumper().onTrue(this.m_algaeClimbertakePivot.setVoltageCommand(8));
    this.OPERATOR.leftBumper().onFalse(this.m_algaeClimbertakePivot.setVoltageCommand(0));

    // * Manuel Climbertake
    // Ready
    this.OPERATOR.leftButton().onTrue(this.m_algaeClimbertakeRollers.setInCommand());
    this.OPERATOR.leftButton().onFalse(this.m_algaeClimbertakeRollers.stopCommand());
    
    this.OPERATOR.topButton().onTrue(this.m_algaeClimbertakeRollers.setOutCommand());
    this.OPERATOR.topButton().onFalse(this.m_algaeClimbertakeRollers.stopCommand());

    // * Coral Intake
    // Ready
    this.OPERATOR.leftButton().onTrue(this.m_coralIntakeRollers.setInCommand());
    this.OPERATOR.leftButton().onFalse(this.m_coralIntakeRollers.stopCommand());

    this.OPERATOR.topButton().onTrue(this.m_coralIntakeRollers.setOutCommand());
    this.OPERATOR.topButton().onFalse(this.m_coralIntakeRollers.stopCommand());

    // * Wrist
    // Ready
    this.OPERATOR.bottomButton().onTrue(this.m_coralIntakeWrist.setSetpointCommand(WristPosition.PERPENDICULAR));
    this.OPERATOR.rightButton().onTrue(this.m_coralIntakeWrist.setSetpointCommand(WristPosition.PARALLEL));

    // * Arm
    // Ready
    this.OPERATOR.leftTrigger().onTrue(this.m_coralIntakeArm.setSetpointCommand(ArmPosition.HORIZONTAL));
    this.OPERATOR.rightTrigger().onTrue(this.m_coralIntakeArm.setSetpointCommand(ArmPosition.HIGH));

    // * Selected level bindings
    // this.OPERATOR.povDown().onTrue(m_elevator.setSetpointCommand(ReefLevel.L1.getElevatorPosition()));
    // this.OPERATOR.povLeft().onTrue(m_elevator.setSetpointCommand(ReefLevel.L2.getElevatorPosition()));
    // this.OPERATOR.povRight().onTrue(m_elevator.setSetpointCommand(ReefLevel.L3.getElevatorPosition()));
    // this.OPERATOR.povUp().onTrue(m_elevator.setSetpointCommand(ReefLevel.L4.getElevatorPosition()));

    this.OPERATOR.povDown().onTrue(ScoringCommands.scorePositionCommand(RobotState.L1, m_elevator, m_coralIntakeArm, m_coralIntakeWrist));
    this.OPERATOR.povLeft().onTrue(ScoringCommands.scorePositionCommand(RobotState.L2, m_elevator, m_coralIntakeArm, m_coralIntakeWrist));
    this.OPERATOR.povRight().onTrue(ScoringCommands.scorePositionCommand(RobotState.L3, m_elevator, m_coralIntakeArm, m_coralIntakeWrist));
    this.OPERATOR.povUp().onTrue(ScoringCommands.scorePositionCommand(RobotState.L4, m_elevator, m_coralIntakeArm, m_coralIntakeWrist));
  }

  public Command getAutonomousCommand() {
    return m_autonomousChooser.getSelected();
  }
}
