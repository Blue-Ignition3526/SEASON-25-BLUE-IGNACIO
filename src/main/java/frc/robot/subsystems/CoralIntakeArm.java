package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmPivotConstants;

// TODO: CHECK LOGIC REALLY CLOSELY
public class CoralIntakeArm extends SubsystemBase {
  // * Setpoints
  public static enum ArmPosition {
    // 0 corelates to horizontal
    // 90 corelates to vertical
    // ! BE MINDFUL OF MECHANICAL LIMITS
    L1(Degrees.of(0)),
    L2(Degrees.of(40)),
    L3(Degrees.of(40)),
    L4(Degrees.of(60));

    private Angle angle;
    private ArmPosition(Angle angle) {
      this.angle = angle;
    }

    public Angle getAngle() {
      return angle;
    }
  }

  // Motor
  private final SparkMax motor;

  // Config
  private final SparkMaxConfig motorConfig;

  // Encoder
  private final RelativeEncoder encoder;

  // Gyro
  private final Pigeon2 gyro;
  private final StatusSignal<Double> gVecX;
  private final StatusSignal<Double> gVecY;
  private final StatusSignal<Double> gVecZ;

  // State
  private Angle setpoint;
  private boolean pidEnabled = false;

  // Alerts
  private final Alert alert_motorUnreachable = new Alert(getName() + " motor unreachable", AlertType.kError);
  private final Alert alert_gyroUnreachable = new Alert(getName() + " gyro unreachable", AlertType.kError);
  private final Alert alert_pidDisabled = new Alert(getName() + " pid disabled", AlertType.kWarning);

  // Device check notifier
  private final Notifier deviceCheckNotifier = new Notifier(this::deviceCheck);

  public CoralIntakeArm() {
    // * Motor and encoder
    // Create motor
    this.motor = new SparkMax(ArmPivotConstants.kArmPivotMotorID, MotorType.kBrushless);

    // Motor config
    this.motorConfig = new SparkMaxConfig();
    this.motorConfig
      .idleMode(IdleMode.kBrake)
      .openLoopRampRate(ArmPivotConstants.kArmPivotMotorRampRate)
      .closedLoopRampRate(ArmPivotConstants.kArmPivotMotorRampRate)
      .smartCurrentLimit(ArmPivotConstants.kArmPivotMotorCurrentLimit)
      .voltageCompensation(12);

    this.motorConfig.encoder
      .positionConversionFactor(1. / 240.);
      
    // Get and configure encoder
    this.encoder = motor.getEncoder();
    
    // Apply motor config
    motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    //* Gyro
    this.gyro = new Pigeon2(ArmPivotConstants.kGyroID);

    this.gVecX = this.gyro.getGravityVectorX();
    this.gVecY = this.gyro.getGravityVectorY();
    this.gVecZ = this.gyro.getGravityVectorZ();

    // * Wait for angle to become available to reset
    System.out.println("Waiting for " + getName() + " gyro to become available...");
    StatusCode absoluteEncoderStatusCode = this.gVecX.waitForUpdate(20).getStatus();

    if (absoluteEncoderStatusCode.isOK()) {
        System.out.println(getName() + " gyro is available, resetting position...");
        resetAngle();
    } else {
        DriverStation.reportError("Failed to get " + getName() + " gyro position. Status: " + absoluteEncoderStatusCode, false);
        System.out.println("Please check " + getName() + " gyro connection.");
        System.out.println("If problem persists please align the wheels manually and restart the robot.");
        System.out.println("Setting current position as 0.");
        encoder.setPosition(0);
        alert_gyroUnreachable.set(true);
    }

    // * Setpoint
    this.setpoint = getAngle();

    // * Log PID
    SmartDashboard.putData("ArmPivot/PID", ArmPivotConstants.kArmPivotPIDController);

    //* Device check
    deviceCheckNotifier.startPeriodic(Constants.deviceCheckPeriod);
  }

  private void deviceCheck() {
    try {
      motor.getFirmwareVersion();
      alert_motorUnreachable.set(false);
    } catch(Exception _e) {
      alert_motorUnreachable.set(true);
      DriverStation.reportError(alert_motorUnreachable.getText(), false);
    }

    try {
      // TODO: this doesnt work
      gyro.getRoll();
      alert_gyroUnreachable.set(false);
    } catch(Exception e) {
      alert_gyroUnreachable.set(true);
      DriverStation.reportError(alert_gyroUnreachable.getText(), false);
    }

    alert_pidDisabled.set(!pidEnabled);
  }

  /*
   * Resets the angle
   */
  public void resetAngle() {
    encoder.setPosition(getGyroAngle().in(Rotations));
  }

  public Command resetAngleCommand() {
    return runOnce(this::resetAngle);
  }

  /**
   * Gets the current angle of the arm from 0 to 2pi
   */
  public Angle getGyroAngle() {
    double xVec = gVecX.refresh().getValueAsDouble();
    double yVec = gVecZ.refresh().getValueAsDouble();

    return Radians.of(Math.atan2(yVec, xVec) + Math.PI);
  }

  /**
   * Dev interface for quickly switching out feedback devices
   * @return
   */
  public Angle getAngle() {
    return Rotations.of(encoder.getPosition()).plus(Degrees.of(66));
  }

  /**
   * Resets the PID Controller
   */
  public void resetPID() {
    ArmPivotConstants.kArmPivotPIDController.reset(getAngle().in(Radians));
  }

  /**
   * Resets the PID controller
   * @return
   */
  public Command resetPIDCommand() {
    return runOnce(this::resetPID);
  }

  /**
   * Sets the setpoint of the PID controller
   * @param setpoint
   */
  public void setSetpoint(Angle setpoint) {
    this.setpoint = Radians.of(MathUtil.clamp(
      setpoint.in(Radians),
      ArmPivotConstants.kMinAngle.in(Radians),
      ArmPivotConstants.kMaxAngle.in(Radians)
    ));
  }

  /**
   * Sets the setpoint of the PID controller
   * @param setpoint
   * @return
   */
  public Command setSetpointCommand(Angle setpoint) {
    return runOnce(() -> setSetpoint(setpoint));
  }

  /**
   * Returns wether or not the PID controller is at the setpoint
   * @return
   */
  public boolean atSetpoint() {
    return ArmPivotConstants.kArmPivotPIDController.atSetpoint();
  }

  @Override
  public void periodic() {
    double currentAngleRad = getAngle().in(Radians);
    double setpointAngleRad = setpoint.in(Radians);

    double resultVolts = ArmPivotConstants.kArmPivotPIDController.calculate(currentAngleRad, setpointAngleRad);

    if (pidEnabled) motor.setVoltage(resultVolts);
    else motor.setVoltage(0);

    // Gyro stats
    SmartDashboard.putNumber("ArmPivot/Pigeon/gVecX", gVecX.getValueAsDouble());
    SmartDashboard.putNumber("ArmPivot/Pigeon/gVecY", gVecY.getValueAsDouble());
    SmartDashboard.putNumber("ArmPivot/Pigeon/gVecZ", gVecZ.getValueAsDouble());

    SmartDashboard.putNumber("ArmPivot/Pigeon/Roll", gyro.getRoll().getValueAsDouble());
    SmartDashboard.putNumber("ArmPivot/Pigeon/Pitch", gyro.getPitch().getValueAsDouble());
    SmartDashboard.putNumber("ArmPivot/Pigeon/Yaw", gyro.getYaw().getValueAsDouble());

    SmartDashboard.putNumber("ArmPivot/Pigeon/gAngle", getGyroAngle().in(Degrees));

    SmartDashboard.putNumber("ArmPivot/SetpointAngle", Math.toDegrees(setpointAngleRad));
    SmartDashboard.putNumber("ArmPivot/CurrentAngle", Math.toDegrees(currentAngleRad));
    SmartDashboard.putNumber("ArmPivot/OutputVolts", resultVolts);
  }
}