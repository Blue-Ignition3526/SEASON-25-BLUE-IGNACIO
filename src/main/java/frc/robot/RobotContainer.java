package frc.robot;

import java.util.HashMap;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.DriveFeedforwards;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ArmPivotConstants;
import frc.robot.Constants.SwerveDriveConstants;
import frc.robot.Constants.RobotState;
import frc.robot.commands.DriveSwerve;
import frc.robot.commands.CompoundCommands.ScoringCommands;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.CoralIntakeRollers;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CoralIntakeArm;
import frc.robot.subsystems.SwerveModule;
import frc.robot.subsystems.CoralIntakeArm.ArmPosition;
import frc.robot.subsystems.CoralIntakeWrist.WristPosition;
import frc.robot.subsystems.Elevator.ElevatorPosition;
import frc.robot.subsystems.CoralIntakeWrist;
import frc.robot.subsystems.Gyro.Gyro;
import frc.robot.subsystems.Gyro.GyroIOPigeon;
import frc.robot.subsystems.SwerveDrive.SwerveDrive;
import frc.robot.subsystems.SwerveDrive.SwerveDriveIOReal;
import frc.robot.subsystems.SwerveDrive.SwerveDriveIOSim;
import lib.Elastic;
import lib.Elastic.Notification;
import lib.Elastic.Notification.NotificationLevel;
import lib.BlueShift.commands.LogCommand;
import lib.BlueShift.control.CustomController;
import lib.BlueShift.control.CustomController.CustomControllerType;
import lib.BlueShift.odometry.swerve.BlueShiftOdometry;
import lib.BlueShift.odometry.vision.camera.LimelightOdometryCamera;
import lib.BlueShift.odometry.vision.camera.VisionOdometryFilters;
import lib.BlueShift.control.SpeedAlterator;
import frc.robot.speedAlterators.*;

// TODO: que cuando el pigeon sienta tilt, que se retaiga en elevador
// TODO: QUE NO BAJE EL ELEVADOR DEL 0
public class RobotContainer {
  // * Controllers
  private final CustomController DRIVER = new CustomController(0, Robot.isReal() ? CustomControllerType.XBOX : CustomControllerType.PS5);
  private final CustomController OPERATOR = new CustomController(1, CustomControllerType.PS5);

  // Swerve Drive
  private final SwerveDrive m_swerveDrive;

  // Speed alterators
  private final SpeedAlterator m_speedAlterator_turn180;
  private final SpeedAlterator m_speedAlterator_lookAt;
  private final SpeedAlterator m_speedAlterator_LookAtNearestStation;
  private final SpeedAlterator m_speedAlterator_AlignToNearestBranch;

  // * Elevator
  private final Elevator m_elevator;

  // * Coral intake
  private final CoralIntakeWrist m_coralIntakeWrist;
  private final CoralIntakeArm m_coralIntakeArm;
  private final CoralIntakeRollers m_coralIntakeRollers;
  
  // * Odometry and Vision
  private final LimelightOdometryCamera m_limelight3G;
  private final BlueShiftOdometry m_odometry;
  private final double m_visionPeriod = 0.02;

  // * Autonomous
  private final SendableChooser<Command> m_autonomousChooser;

  private final Climber m_climber;

  // * Robot state
  public RobotState m_robotState = RobotState.HOME;

  public RobotContainer() {
    // * Swerve Drive
    if (Robot.isReal()) {
      this.m_swerveDrive = new SwerveDrive(new SwerveDriveIOReal(
        new SwerveModule(Constants.SwerveDriveConstants.SwerveModuleConstants.kFrontLeftOptions),
        new SwerveModule(Constants.SwerveDriveConstants.SwerveModuleConstants.kFrontRightOptions),
        new SwerveModule(Constants.SwerveDriveConstants.SwerveModuleConstants.kBackLeftOptions),
        new SwerveModule(Constants.SwerveDriveConstants.SwerveModuleConstants.kBackRightOptions),
        new Gyro(new GyroIOPigeon(Constants.SwerveDriveConstants.kGyroDevice))
      ));
    } else {
      this.m_swerveDrive = new SwerveDrive(new SwerveDriveIOSim());
    }

    // * Elevator
    this.m_elevator = new Elevator();
    
    // * Coral intake
    m_coralIntakeWrist = new CoralIntakeWrist();
    m_coralIntakeArm = new CoralIntakeArm();
    m_coralIntakeRollers = new CoralIntakeRollers();
    m_climber = new Climber();

    // * Odometry and Vision
    this.m_limelight3G = new LimelightOdometryCamera(Constants.Vision.Limelight3G.kName, false, VisionOdometryFilters::visionFilter);
    this.m_odometry = new BlueShiftOdometry(
      Constants.SwerveDriveConstants.PhysicalModel.kDriveKinematics, 
      m_swerveDrive::getHeading,
      m_swerveDrive::getModulePositions,
      new Pose2d(),
      m_visionPeriod,
      m_limelight3G
    );
    this.m_limelight3G.enable();
    this.m_odometry.startVision();

    // * Speed alterators
    this.m_speedAlterator_turn180 = new Turn180(m_odometry::getEstimatedPosition);
    this.m_speedAlterator_lookAt = new LookController(this.m_swerveDrive::getHeading, this.DRIVER::getRightX, this.DRIVER::getRightY, Constants.SwerveDriveConstants.kJoystickDeadband);
    this.m_speedAlterator_LookAtNearestStation = new LookAtNearestStation(m_odometry::getEstimatedPosition);
    this.m_speedAlterator_AlignToNearestBranch = new AlignToNearestBranch(m_odometry::getEstimatedPosition, this.DRIVER.rightBumper()::getAsBoolean, this.DRIVER::getLeftY, this.DRIVER::getLeftX);
    
    // * Autonomous
    // Register commands
    NamedCommands.registerCommand("L1", ScoringCommands.scorePositionAutoCommand(RobotState.L1, m_elevator, m_coralIntakeArm, m_coralIntakeWrist));
    NamedCommands.registerCommand("L2", ScoringCommands.scorePositionAutoCommand(RobotState.L2, m_elevator, m_coralIntakeArm, m_coralIntakeWrist));
    NamedCommands.registerCommand("L3", ScoringCommands.scorePositionAutoCommand(RobotState.L3, m_elevator, m_coralIntakeArm, m_coralIntakeWrist));
    NamedCommands.registerCommand("L4", ScoringCommands.scorePositionAutoCommand(RobotState.L4, m_elevator, m_coralIntakeArm, m_coralIntakeWrist));

    NamedCommands.registerCommand("Home", ScoringCommands.scorePositionAutoCommand(RobotState.HOME, m_elevator, m_coralIntakeArm, m_coralIntakeWrist));

    NamedCommands.registerCommand("L1-ALT", ScoringCommands.scorePositionAutoCommandWithWait(RobotState.L1, m_elevator, m_coralIntakeArm, m_coralIntakeWrist));
    NamedCommands.registerCommand("L2-ALT", ScoringCommands.scorePositionAutoCommandWithWait(RobotState.L2, m_elevator, m_coralIntakeArm, m_coralIntakeWrist));
    NamedCommands.registerCommand("L3-ALT", ScoringCommands.scorePositionAutoCommandWithWait(RobotState.L3, m_elevator, m_coralIntakeArm, m_coralIntakeWrist));
    NamedCommands.registerCommand("L4-ALT", ScoringCommands.scorePositionAutoCommandWithWait(RobotState.L4, m_elevator, m_coralIntakeArm, m_coralIntakeWrist));

    NamedCommands.registerCommand("Score-L1", ScoringCommands.scoreCommand(RobotState.L1, m_elevator, m_coralIntakeArm, m_coralIntakeWrist, m_coralIntakeRollers));
    NamedCommands.registerCommand("Score-L2", ScoringCommands.scoreCommand(RobotState.L2, m_elevator, m_coralIntakeArm, m_coralIntakeWrist, m_coralIntakeRollers));
    NamedCommands.registerCommand("Score-L3", ScoringCommands.scoreCommand(RobotState.L3, m_elevator, m_coralIntakeArm, m_coralIntakeWrist, m_coralIntakeRollers));
    NamedCommands.registerCommand("Score-L4", ScoringCommands.scoreCommand(RobotState.L4, m_elevator, m_coralIntakeArm, m_coralIntakeWrist, m_coralIntakeRollers));

    NamedCommands.registerCommand("Eat", new ParallelCommandGroup(
      ScoringCommands.scorePositionAutoCommand(RobotState.SOURCE, m_elevator, m_coralIntakeArm, m_coralIntakeWrist),
      m_coralIntakeRollers.intakeUntilPieceDetected()
    ));

    NamedCommands.registerCommand("Intake", new ParallelCommandGroup(
      ScoringCommands.scorePositionAutoCommand(RobotState.SOURCE, m_elevator, m_coralIntakeArm, m_coralIntakeWrist),
      new InstantCommand(m_coralIntakeRollers::setIn)
    ));

    // Robot config
    RobotConfig ppRobotConfig = null;
    try{
      ppRobotConfig = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      Elastic.sendNotification(new Notification(NotificationLevel.ERROR, "ERROR! COULD NOT LOAD PP ROBOT CONFIG", e.getMessage()));
      DriverStation.reportError("ERROR! COULD NOT LOAD PP ROBOT CONFIG", e.getStackTrace());
    }

    AutoBuilder.configure(
      m_odometry::getEstimatedPosition,
      m_odometry::resetPosition,
      m_swerveDrive::getRobotRelativeChassisSpeeds,
      (ChassisSpeeds speeds, DriveFeedforwards ff) -> m_swerveDrive.driveRobotRelative(speeds),
      new PPHolonomicDriveController(
        SwerveDriveConstants.AutonomousConstants.kTranslatePIDConstants,
        SwerveDriveConstants.AutonomousConstants.kRotatePIDConstants
      ),
      ppRobotConfig,
      () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
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
    // SmartDashboard.putData("Climbertake/Pivot/IntakeAngleCommand", m_algaeClimbertakePivot.setSetpointCommand(Constants.ClimbertakeConstants.Pivot.kIntakeAngle).ignoringDisable(true));
    // SmartDashboard.putData("Climbertake/Pivot/StoreAngleCommand", m_algaeClimbertakePivot.setSetpointCommand(Constants.ClimbertakeConstants.Pivot.kStoreAngle).ignoringDisable(true));
    // SmartDashboard.putData("Climbertake/Pivot/ClimbHighAngleCommand", m_algaeClimbertakePivot.setSetpointCommand(Constants.ClimbertakeConstants.Pivot.kClimbHighAngle).ignoringDisable(true));
    // SmartDashboard.putData("Climbertake/Pivot/ClimbLowAngleCommand", m_algaeClimbertakePivot.setSetpointCommand(Constants.ClimbertakeConstants.Pivot.kClimbLowAngle).ignoringDisable(true));

    // Wrist
    SmartDashboard.putData("Wrist/Perpendicular", m_coralIntakeWrist.setSetpointCommand(WristPosition.PERPENDICULAR).ignoringDisable(true));
    SmartDashboard.putData("Wrist/Parallel", m_coralIntakeWrist.setSetpointCommand(WristPosition.PARALLEL).ignoringDisable(true));

    // Arm pivot
    SmartDashboard.putData("ArmPivot/ResetAngle", m_coralIntakeArm.resetAngleCommand().ignoringDisable(true));
    SmartDashboard.putData("ArmPivot/LowAngle", m_coralIntakeArm.setSetpointCommand(ArmPivotConstants.kLowAngle).ignoringDisable(true));
    SmartDashboard.putData("ArmPivot/MidAngle", m_coralIntakeArm.setSetpointCommand(ArmPivotConstants.kMidAngle).ignoringDisable(true));
    SmartDashboard.putData("ArmPivot/HighAngle", m_coralIntakeArm.setSetpointCommand(ArmPivotConstants.kHighAngle).ignoringDisable(true));

    SmartDashboard.putData("Elevator/ResetPosition", m_elevator.resetElevatorPositionCommand().ignoringDisable(true));

    SmartDashboard.putData("Dev/ResetOdo", new InstantCommand(() -> m_odometry.resetPosition(new Pose2d(new Translation2d(4, 4), new Rotation2d()))));
    SmartDashboard.putData("Dev/TranslationXPID", Constants.SwerveDriveConstants.PoseControllers.translationXPID);
    SmartDashboard.putData("Dev/TranslationYPID", Constants.SwerveDriveConstants.PoseControllers.translationYPID);
    SmartDashboard.putData("Dev/TranslationRotPID", Constants.SwerveDriveConstants.PoseControllers.rotationPID);

    // ! BIND PID RESETS
    Trigger enabledTrigger = new Trigger(DriverStation::isEnabled);
    enabledTrigger.onTrue(new ParallelCommandGroup(
      m_coralIntakeWrist.resetPIDCommand(),
      m_coralIntakeArm.resetPIDCommand(),
      new InstantCommand(m_odometry::setVisionPose),
      new LogCommand("Enabled!")
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
        () -> DRIVER.getLeftTrigger() - DRIVER.getRightTrigger(),
        () -> !DRIVER.bottomButton().getAsBoolean()
      )
    );
    
    // * Align to reef alterator
    this.DRIVER.rightBumper().whileTrue(m_swerveDrive.enableSpeedAlteratorCommand(m_speedAlterator_AlignToNearestBranch));
    this.DRIVER.rightBumper().onFalse(m_swerveDrive.disableSpeedAlteratorCommand());

    this.DRIVER.leftBumper().whileTrue(m_swerveDrive.enableSpeedAlteratorCommand(m_speedAlterator_AlignToNearestBranch));
    this.DRIVER.leftBumper().onFalse(m_swerveDrive.disableSpeedAlteratorCommand());

    // * Reset heading with right stick button
    //TODO: think of a better button to bind this to
    this.DRIVER.rightStickButton().onTrue(this.m_swerveDrive.zeroHeadingCommand());

    this.DRIVER.startButton().onTrue(m_climber.setVoltCommand(7));
    this.DRIVER.startButton().onFalse(m_climber.setVoltCommand(0));
    this.DRIVER.backButton().onTrue(m_climber.setVoltCommand(-7));
    this.DRIVER.backButton().onFalse(m_climber.setVoltCommand(0));

    // * Driver Coral intake
    this.DRIVER.leftButton().onTrue(new ParallelCommandGroup(
      ScoringCommands.scorePositionCommand(RobotState.SOURCE, m_elevator, m_coralIntakeArm, m_coralIntakeWrist),
      m_coralIntakeRollers.intakeUntilPieceDetected()
    ));
    this.DRIVER.leftButton().onFalse(new ParallelCommandGroup(
      ScoringCommands.scorePositionCommand(RobotState.HOME, m_elevator, m_coralIntakeArm, m_coralIntakeWrist),
      m_coralIntakeRollers.intakeUntilPieceDetected()
    ));

    // * Driver score
    this.DRIVER.topButton().onTrue(ScoringCommands.scoreCommand(ScoringCommands.StateMachine.getInstance().getState(), m_elevator, m_coralIntakeArm, m_coralIntakeWrist, m_coralIntakeRollers));

    // ! OPERATOR BINDINGS
    // * Manuel Elevator
    // Options
    this.OPERATOR.startButton().whileTrue(m_elevator.setVoltageCommand(6));
    this.OPERATOR.startButton().onFalse(m_elevator.stopCommand());

    // Share
    this.OPERATOR.backButton().whileTrue(m_elevator.setVoltageCommand(-6));
    this.OPERATOR.backButton().onFalse(m_elevator.stopCommand());

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

    // * Autoalign
    this.OPERATOR.rightStickButton().whileTrue(m_swerveDrive.enableSpeedAlteratorCommand(m_speedAlterator_LookAtNearestStation));
    this.OPERATOR.rightStickButton().onFalse(m_swerveDrive.disableSpeedAlteratorCommand());

    // * Selected level bindings
    //this.OPERATOR.povDown().onTrue(new InstantCommand(() -> m_robotState = RobotState.L1));
    //this.OPERATOR.povLeft().onTrue(new InstantCommand(() -> m_robotState = RobotState.L2));
    //this.OPERATOR.povRight().onTrue(new InstantCommand(() -> m_robotState = RobotState.L3));
    //this.OPERATOR.povUp().onTrue(new InstantCommand(() -> m_robotState = RobotState.L4));

    this.OPERATOR.povDown().onTrue(ScoringCommands.scorePositionCommand(RobotState.L1, m_elevator, m_coralIntakeArm, m_coralIntakeWrist));
    this.OPERATOR.povLeft().onTrue(ScoringCommands.scorePositionCommand(RobotState.L2, m_elevator, m_coralIntakeArm, m_coralIntakeWrist));
    this.OPERATOR.povRight().onTrue(ScoringCommands.scorePositionCommand(RobotState.L3, m_elevator, m_coralIntakeArm, m_coralIntakeWrist));
    this.OPERATOR.povUp().onTrue(ScoringCommands.scorePositionCommand(RobotState.L4, m_elevator, m_coralIntakeArm, m_coralIntakeWrist));
  }

  public Command getAutonomousCommand() {
    return m_autonomousChooser.getSelected();
  }
}
