package frc.robot;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Degrees;
import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.ArmPivotConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.commands.DriveSwerve;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.IntakeCoral;
import frc.robot.subsystems.ArmPivot;
import frc.robot.subsystems.ClimbertakePivot;
import frc.robot.subsystems.ClimbertakeRollers;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.SwerveModule;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.Gyro.Gyro;
import frc.robot.subsystems.Gyro.GyroIOPigeon;
import lib.BlueShift.control.CustomController;
import lib.BlueShift.control.CustomController.CustomControllerType;
import lib.BlueShift.odometry.swerve.BlueShiftOdometry;
import lib.BlueShift.odometry.vision.camera.LimelightOdometryCamera;
import lib.BlueShift.odometry.vision.camera.VisionOdometryFilters;
import lib.BlueShift.control.SpeedAlterator;
import frc.robot.speedAlterators.*;

public class RobotContainer {
  private final int m_driverControllerPort = 0;
  private final CustomController m_driverControllerCustom = new CustomController(m_driverControllerPort, CustomControllerType.XBOX);

  // * Gyro
  private final Gyro m_gyro;
  
  // * Swerve
  private final SwerveModule frontLeft = new SwerveModule(Constants.SwerveDriveConstants.SwerveModuleConstants.kFrontLeftOptions);
  private final SwerveModule frontRight = new SwerveModule(Constants.SwerveDriveConstants.SwerveModuleConstants.kFrontRightOptions);
  private final SwerveModule backLeft = new SwerveModule(Constants.SwerveDriveConstants.SwerveModuleConstants.kBackLeftOptions);
  private final SwerveModule backRight = new SwerveModule(Constants.SwerveDriveConstants.SwerveModuleConstants.kBackRightOptions);
  private final SwerveDrive m_swerveDrive;

  // * Elevator
  private final Elevator m_elevator;

  private final SendableChooser<Command> autonomousChooser;

  // * Climbertake
  private final ClimbertakePivot m_climbertakePivot;
  private final ClimbertakeRollers m_intakeAlgea;

  // * Coral intake
  private final Wrist m_wrist;
  private final ArmPivot m_armPivot;
  private final IntakeCoral m_intakeCoral;
  
  // * Speed alterators
  private final SpeedAlterator turn180;
  private final SpeedAlterator lookAt;
  private final SpeedAlterator goTo0;
  private final SpeedAlterator gotTo1;

  // Odometry and Vision
  private final LimelightOdometryCamera m_limelight3G;
  private final BlueShiftOdometry m_odometry;
  private final double m_visionPeriod = 0.1;
  
  public RobotContainer() {
    // * Gyro
    m_gyro = new Gyro(new GyroIOPigeon(Constants.SwerveDriveConstants.kGyroDevice));

    // * Swerve
    m_swerveDrive = new SwerveDrive(frontLeft, frontRight, backLeft, backRight, m_gyro);

    // * Speed alterators
    this.turn180 = new Turn180(this.m_swerveDrive);
    this.lookAt = new LookController(this.m_gyro::getYaw, this.m_driverControllerCustom::getRightX, this.m_driverControllerCustom::getRightY, 0.1);
    this.goTo0 = new GoToPose(this.m_swerveDrive::getPose, new Pose2d(0, 0, new Rotation2d()));
    this.gotTo1 = new GoToPose(this.m_swerveDrive::getPose, new Pose2d(0, 3, Rotation2d.fromDegrees(180)));

    autonomousChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("ZeroHeading", m_swerveDrive.zeroHeadingCommand());
    SmartDashboard.putData("Autonomous", autonomousChooser);
    SmartDashboard.putData("Reset Odometry", m_swerveDrive.resetPoseCommand());

    // Odometry and Vision
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

    SmartDashboard.putData("SwerveDrive/ResetTurningEncoders", new InstantCommand(m_swerveDrive::resetTurningEncoders).ignoringDisable(true));

    // * Elevator
    this.m_elevator = new Elevator();

    // * Climbertake
    m_climbertakePivot = new ClimbertakePivot();
    m_intakeAlgea = new ClimbertakeRollers();

    // * Coral intake
    m_wrist = new Wrist();
    m_armPivot = new ArmPivot();
    m_intakeCoral = new IntakeCoral();

    // * Dashboard testing commands
    // Elevator
    SmartDashboard.putData("Elevator/10", m_elevator.setSetpointCommand(Inches.of(10)).ignoringDisable(true));
    SmartDashboard.putData("Elevator/0", m_elevator.setSetpointCommand(Inches.of(0)).ignoringDisable(true));
    SmartDashboard.putData("Elevator/50", m_elevator.setSetpointCommand(Inches.of(50)).ignoringDisable(true));
    SmartDashboard.putData("Elevator/-10", m_elevator.setSetpointCommand(Inches.of(-10)).ignoringDisable(true));

    // Climbertake pivot
    SmartDashboard.putData("Climbertake/Pivot/IntakeAngleCommand", m_climbertakePivot.setSetpointCommand(Constants.ClimbertakeConstants.Pivot.kIntakeAngle).ignoringDisable(true));
    SmartDashboard.putData("Climbertake/Pivot/StoreAngleCommand", m_climbertakePivot.setSetpointCommand(Constants.ClimbertakeConstants.Pivot.kStoreAngle).ignoringDisable(true));
    SmartDashboard.putData("Climbertake/Pivot/ClimbHighAngleCommand", m_climbertakePivot.setSetpointCommand(Constants.ClimbertakeConstants.Pivot.kClimbHighAngle).ignoringDisable(true));
    SmartDashboard.putData("Climbertake/Pivot/ClimbLowAngleCommand", m_climbertakePivot.setSetpointCommand(Constants.ClimbertakeConstants.Pivot.kClimbLowAngle).ignoringDisable(true));

    // Wrist
    SmartDashboard.putData("Wrist/Perpendicular", m_wrist.setSetpointCommand(WristConstants.kPerpendicular).ignoringDisable(true));
    SmartDashboard.putData("Wrist/Parallel", m_wrist.setSetpointCommand(WristConstants.kParallel).ignoringDisable(true));

    // Arm pivot
    SmartDashboard.putData("ArmPivot/ResetAngle", m_armPivot.resetAngleCommand().ignoringDisable(true));
    SmartDashboard.putData("ArmPivot/LowAngle", m_armPivot.setSetpointCommand(ArmPivotConstants.kLowAngle).ignoringDisable(true));
    SmartDashboard.putData("ArmPivot/MidAngle", m_armPivot.setSetpointCommand(ArmPivotConstants.kMidAngle).ignoringDisable(true));
    SmartDashboard.putData("ArmPivot/HighAngle", m_armPivot.setSetpointCommand(ArmPivotConstants.kHighAngle).ignoringDisable(true));

    configureBindings();
  }

  private void configureBindings() {
    this.m_swerveDrive.setDefaultCommand(new DriveSwerve(
        m_swerveDrive,
        () -> -m_driverControllerCustom.getLeftY(),
        () -> -m_driverControllerCustom.getLeftX(),
        () -> -m_driverControllerCustom.getRightX(),
        () -> true
      )
    );

    this.m_driverControllerCustom.rightButton().onTrue(this.m_swerveDrive.zeroHeadingCommand());
    this.m_driverControllerCustom.leftButton().onTrue(this.m_swerveDrive.resetPoseCommand());

    // this.m_driverControllerCustom.rightBumper().onTrue(this.m_climbertakePivot.setVoltageCommand(8));
    // this.m_driverControllerCustom.rightBumper().onFalse(this.m_climbertakePivot.setVoltageCommand(0));
// 
    // this.m_driverControllerCustom.leftBumper().onTrue(this.m_climbertakePivot.setVoltageCommand(-6));
    // this.m_driverControllerCustom.leftBumper().onFalse(this.m_climbertakePivot.setVoltageCommand(0));



    this.m_driverControllerCustom.leftButton().onTrue(this.m_intakeCoral.setInCommand());
    this.m_driverControllerCustom.leftButton().onFalse(this.m_intakeCoral.stopCommand());
    this.m_driverControllerCustom.topButton().onTrue(this.m_intakeCoral.setOutCommand());
    this.m_driverControllerCustom.topButton().onFalse(this.m_intakeCoral.stopCommand());

    this.m_driverControllerCustom.leftButton().onTrue(this.m_intakeAlgea.setInCommand());
    this.m_driverControllerCustom.leftButton().onFalse(this.m_intakeAlgea.stopCommand());
    this.m_driverControllerCustom.topButton().onTrue(this.m_intakeAlgea.setOutCommand());
    this.m_driverControllerCustom.topButton().onFalse(this.m_intakeAlgea.stopCommand());


    // this.m_driverControllerCustom.rightBumper().onTrue(m_swerveDrive.enableSpeedAlteratorCommand(turn180));
    // this.m_driverControllerCustom.leftBumper().onTrue(m_swerveDrive.enableSpeedAlteratorCommand(lookAt));
    // this.m_driverControllerCustom.bottomButton().onTrue(m_swerveDrive.enableSpeedAlteratorCommand(goTo0));
    // this.m_driverControllerCustom.topButton().onTrue(m_swerveDrive.enableSpeedAlteratorCommand(gotTo1));
    // this.m_driverControllerCustom.rightBumper().onFalse(m_swerveDrive.disableSpeedAlteratorCommand());
    // this.m_driverControllerCustom.leftBumper().onFalse(m_swerveDrive.disableSpeedAlteratorCommand());
    // this.m_driverControllerCustom.bottomButton().onFalse(m_swerveDrive.disableSpeedAlteratorCommand());
    // this.m_driverControllerCustom.topButton().onFalse(m_swerveDrive.disableSpeedAlteratorCommand());
  }

  public Command getAutonomousCommand() {
    return autonomousChooser.getSelected();
  }
}
