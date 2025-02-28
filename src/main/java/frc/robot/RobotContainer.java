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
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ArmPivotConstants;
import frc.robot.Constants.SwerveDriveConstants;
import frc.robot.Constants.FieldConstants.ReefLevel;
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

// TODO: PID DISABLED BEHAVIORS IN SUBSYSTEMS ( setvoltage disabled PID, setpoint enables pid if it has an encoder )
public class RobotContainer {
  // * Controllers
  private final CustomController DRIVER = new CustomController(0, CustomControllerType.XBOX);
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

  // * Reef level state machine
  ReefLevel selectedLevel = ReefLevel.HOME;
  ReefLevel lastSelectedLevel = selectedLevel;

  public RobotContainer() {
    // * Gyro
    m_gyro = new Gyro(new GyroIOPigeon(Constants.SwerveDriveConstants.kGyroDevice));

    // * Swerve Drive
    m_swerveDrive = new SwerveDrive(frontLeft, frontRight, backLeft, backRight, m_gyro);

    // * Elevator
    this.m_elevator = new Elevator();
    
    // * Climbertake
    m_climbertakePivot = new AlgaeClimbertakePivot();
    m_intakeAlgea = new AlgaeClimbertakeRollers();
    
    // * Coral intake
    m_wrist = new CoralIntakeWrist();
    m_armPivot = new CoralIntakeArm();
    m_intakeCoral = new CoralIntakeRollers();

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
    this.m_speedAlterator_goTo0 = new GoToPose(m_odometry::getEstimatedPosition, new Pose2d(0, 0, new Rotation2d()));
    this.m_speedAlterator_gotTo1 = new GoToPose(m_odometry::getEstimatedPosition, new Pose2d(0, 3, Rotation2d.fromDegrees(180)));
    this.m_speedAlterator_backUp = new BackUp(-0.1, m_gyro::getHeading);
    
    // * Autonomous
    // Register commands
    NamedCommands.registerCommands(new HashMap<String, Command>(){{
      // put("Score-L1", new SequentialCommandGroup(
      //   new ParallelCommandGroup(
      //     m_elevator.setSetpointCommand(ElevatorPosition.L1),
      //     m_armPivot.setSetpointCommand(ArmPosition.HIGH),
      //     m_wrist.setSetpointCommand(WristPosition.PARALLEL)
      //   ),
      //   new WaitCommand(0.5),
      //   m_intakeCoral.setOutCommand(),
      //   new WaitCommand(1),
      //   m_intakeCoral.stopCommand()
      // ));
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

    SmartDashboard.putData("Elevator/ResetPosition", m_elevator.resetElevatorPositionCommand());

    // ! BIND PID RESETS
    Trigger enabledTrigger = new Trigger(DriverStation::isEnabled);
    enabledTrigger.onTrue(new SequentialCommandGroup(
      m_elevator.resetPIDCommand()
    ));

    // * Add controller bindings
    configureBindings();
  }

  private void configureBindings() {
    // ! DRIVER BINDINGS
    this.m_swerveDrive.setDefaultCommand(new DriveSwerve(
        m_swerveDrive,
        () -> -DRIVER.getLeftY(),
        () -> -DRIVER.getLeftX(),
        () ->  DRIVER.getLeftTrigger() - DRIVER.getRightTrigger(),
        () -> true
      )
    );

    // Look at speed alterator
    Trigger lookAtTrigger = new Trigger(() -> 
      Math.abs(DRIVER.getRightX()) > SwerveDriveConstants.kJoystickDeadband ||
      Math.abs(DRIVER.getRightY()) > SwerveDriveConstants.kJoystickDeadband
    );

    // Bind look at 
    lookAtTrigger.onTrue(m_swerveDrive.enableSpeedAlteratorCommand(m_speedAlterator_lookAt));
    lookAtTrigger.onFalse(m_swerveDrive.disableSpeedAlteratorCommand());

    // Zero heading with right stick but
    this.DRIVER.rightStickButton().onTrue(this.m_swerveDrive.zeroHeadingCommand());

    // ! OPERATOR BINDINGS
    // * Elevator
    // Options
    this.OPERATOR.startButton().whileTrue(m_elevator.setVoltageCommand(10));
    this.OPERATOR.startButton().onFalse(m_elevator.stopCommand());

    // Share
    this.OPERATOR.backButton().whileTrue(m_elevator.setVoltageCommand(-10));
    this.OPERATOR.backButton().onFalse(m_elevator.stopCommand());

    // * Climbertake pivot
    this.OPERATOR.rightBumper().onTrue(this.m_climbertakePivot.setVoltageCommand(-8));
    this.OPERATOR.rightBumper().onFalse(this.m_climbertakePivot.setVoltageCommand(0));
 
    this.OPERATOR.leftBumper().onTrue(this.m_climbertakePivot.setVoltageCommand(8));
    this.OPERATOR.leftBumper().onFalse(this.m_climbertakePivot.setVoltageCommand(0));

    // * Climbertake
    this.OPERATOR.leftButton().onTrue(this.m_intakeAlgea.setInCommand());
    this.OPERATOR.leftButton().onFalse(this.m_intakeAlgea.stopCommand());
    
    this.OPERATOR.topButton().onTrue(this.m_intakeAlgea.setOutCommand());
    this.OPERATOR.topButton().onFalse(this.m_intakeAlgea.stopCommand());

    // * Coral Intake
    this.OPERATOR.leftButton().onTrue(this.m_intakeCoral.setInCommand());
    this.OPERATOR.leftButton().onFalse(this.m_intakeCoral.stopCommand());

    this.OPERATOR.topButton().onTrue(this.m_intakeCoral.setOutCommand());
    this.OPERATOR.topButton().onFalse(this.m_intakeCoral.stopCommand());

    // * Wrist
    this.OPERATOR.bottomButton().onTrue(this.m_wrist.setSetpointCommand(WristPosition.PARALLEL));
    this.OPERATOR.rightButton().onTrue(this.m_wrist.setSetpointCommand(WristPosition.PERPENDICULAR));

    // * Arm
    this.OPERATOR.leftTrigger().onTrue(this.m_armPivot.setSetpointCommand(ArmPosition.HORIZONTAL));
    this.OPERATOR.rightTrigger().onTrue(this.m_armPivot.setSetpointCommand(ArmPosition.HIGH));

    // * Selected level bindings
    // this.OPERATOR.povDown().onTrue(m_elevator.setSetpointCommand(ReefLevel.L1.getElevatorPosition()));
    // this.OPERATOR.povLeft().onTrue(m_elevator.setSetpointCommand(ReefLevel.L2.getElevatorPosition()));
    // this.OPERATOR.povRight().onTrue(m_elevator.setSetpointCommand(ReefLevel.L3.getElevatorPosition()));
    // this.OPERATOR.povUp().onTrue(m_elevator.setSetpointCommand(ReefLevel.L4.getElevatorPosition()));

    this.OPERATOR.povDown().onTrue(ScoringCommands.scorePositionCommand(ReefLevel.L1, m_elevator, m_armPivot, m_wrist));
    this.OPERATOR.povLeft().onTrue(ScoringCommands.scorePositionCommand(ReefLevel.L2, m_elevator, m_armPivot, m_wrist));
    this.OPERATOR.povRight().onTrue(ScoringCommands.scorePositionCommand(ReefLevel.L3, m_elevator, m_armPivot, m_wrist));
    this.OPERATOR.povUp().onTrue(ScoringCommands.scorePositionCommand(ReefLevel.L4, m_elevator, m_armPivot, m_wrist));


  }

  public Command getAutonomousCommand() {
    return m_autonomousChooser.getSelected();
  }
}
