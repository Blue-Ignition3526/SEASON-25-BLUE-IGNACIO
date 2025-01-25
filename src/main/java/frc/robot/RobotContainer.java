package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
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
import lib.team3526.math.BlueMathUtils;
import lib.team3526.control.SpeedAlterator;
import frc.robot.speedAlterators.Turn180;
import frc.robot.speedAlterators.LookTowards;

public class RobotContainer {
  private final int m_driverControllerPort = 0;
  private final CustomController m_driverControllerCustom;
  
  private final SwerveModule frontLeft = new SwerveModule(Constants.SwerveDrive.SwerveModules.kFrontLeftOptions);
  private final SwerveModule frontRight = new SwerveModule(Constants.SwerveDrive.SwerveModules.kFrontRightOptions);
  private final SwerveModule backLeft = new SwerveModule(Constants.SwerveDrive.SwerveModules.kBackLeftOptions);
  private final SwerveModule backRight = new SwerveModule(Constants.SwerveDrive.SwerveModules.kBackRightOptions);

  private final SwerveDrive m_swerveDrive;
  private final Gyro gyro;

  private final SendableChooser<Command> autonomousChooser;

  private final SpeedAlterator alignToSpeakerAlterator;
  private final SpeedAlterator lookAt0;

  public RobotContainer() {
    if (DriverStation.getJoystickIsXbox(m_driverControllerPort)) {
      m_driverControllerCustom = new CustomController(m_driverControllerPort, CustomControllerType.XBOX);
    } else {
      m_driverControllerCustom = new CustomController(m_driverControllerPort, CustomControllerType.PS5);
    }

    gyro = new Gyro(new GyroIOPigeon(Constants.SwerveDrive.kGyroDevice));
    m_swerveDrive = new SwerveDrive(frontLeft, frontRight, backLeft, backRight, gyro);

    this.alignToSpeakerAlterator = new Turn180(this.gyro::getYaw);
    
    this.lookAt0 = new LookTowards(this.gyro::getYaw, () -> BlueMathUtils.mapDouble(this.m_driverControllerCustom.getRightX(), -1, 1, 0, 1));

    SmartDashboard.putData(m_swerveDrive.zeroHeadingCommand());

    autonomousChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Autonomous", autonomousChooser);

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

    this.m_driverControllerCustom.rightBumper().onTrue(m_swerveDrive.enableSpeedAlteratorCommand(alignToSpeakerAlterator));
    this.m_driverControllerCustom.leftBumper().onTrue(m_swerveDrive.enableSpeedAlteratorCommand(lookAt0));
    this.m_driverControllerCustom.rightBumper().onFalse(m_swerveDrive.disableSpeedAlteratorCommand());
    this.m_driverControllerCustom.leftBumper().onFalse(m_swerveDrive.disableSpeedAlteratorCommand());
  }

  public Command getAutonomousCommand() {
    return autonomousChooser.getSelected();
  }
}
