package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.WristConstants;

public class CoralIntakeWrist extends SubsystemBase {
  // * Setpoints
  public static enum WristPosition {
    PARALLEL(Degrees.of(0)),
    PERPENDICULAR(Degrees.of(-90));

    private Angle angle;
    
    private WristPosition(Angle angle) {
      this.angle = angle;
    }

    public Angle getPosition() {
      return angle;
    }
  }

  // * Motor
  private final SparkFlex motor;
  private final SparkFlexConfig motorConfig;

  // * Through bore encoder
  private final DutyCycleEncoder encoder;

  // * Status
  private Angle setpoint = WristPosition.PARALLEL.getPosition();
  private boolean pidEnabled = false;

  // * Alerts
  private final Alert alert_motorUnreachable = new Alert(getName() + " motor unreachable", AlertType.kError);
  private final Alert alert_encoderUnreachable = new Alert(getName() + " encoder unreachable", AlertType.kError);
  private final Alert alert_pidDisabled = new Alert(getName() + " PID disabled.", AlertType.kInfo);

  // * Device check
  private final Notifier deviceCheckNotifier = new Notifier(this::deviceCheck);

  public CoralIntakeWrist() {
    // Create motor
    this.motor = new SparkFlex(WristConstants.kWristMotorID, MotorType.kBrushless);

    // Motor config
    this.motorConfig = new SparkFlexConfig();
    this.motorConfig
      .idleMode(IdleMode.kBrake)
      .openLoopRampRate(WristConstants.kWristMotorRampRate)
      .closedLoopRampRate(WristConstants.kWristMotorRampRate)
      .smartCurrentLimit(WristConstants.kWristMotorCurrentLimit)
      .inverted(true)
      .voltageCompensation(12);

    // Apply motor config
    motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    // Create and configure encoder
    this.encoder = new DutyCycleEncoder(WristConstants.kWristEncoderPort);

    // * Setpoint angle
    //! SETPOINT SET ABOVE
    // this.setpoint = getAngle();
    
    SmartDashboard.putData("Wrist/PID", WristConstants.kWristPIDController);
    
    // Device check
    deviceCheckNotifier.startPeriodic(Constants.deviceCheckPeriod);
  }

  private void deviceCheck() {
    try {
      motor.getFirmwareVersion();
      alert_motorUnreachable.set(false);
    } catch (Exception e) {
      alert_motorUnreachable.set(true);
      DriverStation.reportError(getName() + " encoder unreachable", false);
    }

    if (encoder.isConnected()) {
      alert_encoderUnreachable.set(false);
    } else {
      alert_encoderUnreachable.set(true); 
      DriverStation.reportError(getName() + " piece sensor unreachable", false);
    }
  }

  /**
   * Gets the current angle of the wrist
   */
  public Angle getAngle() {
    return Rotations.of((((encoder.get() + WristConstants.kWristEncoderOffset.in(Rotations)-0.5)%1)+1)%1-0.5);
  }

  /**
   * Resets the PID Controller
   */
  public void resetPID() {
    WristConstants.kWristPIDController.reset(getAngle().in(Radians));
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
  public void setSetpoint(WristPosition setpoint) {
    pidEnabled = true;
    this.setpoint = Radians.of(MathUtil.clamp(
      setpoint.getPosition().in(Radians),
      WristConstants.kMinAngle.in(Radians),
      WristConstants.kMaxAngle.in(Radians)
    ));
  }

  /**
   * Sets the setpoint of the PID controller
   * @param setpoint
   * @return
   */
  public Command setSetpointCommand(WristPosition setpoint) {
    return runOnce(() -> setSetpoint(setpoint));
  }

  /**
   * Returns wether or not the PID controller is at the setpoint
   * @return
   */
  public boolean atSetpoint() {
    return WristConstants.kWristPIDController.atSetpoint();
  }

  @Override
  public void periodic() {
    double currentAngleRad = getAngle().in(Radians);
    // ! CLAMPS ANGLE HERE
    // double setpointAngleRad = MathUtil.clamp(
    //   setpoint.in(Radians),
    //   ClimbertakeConstants.Pivot.kPivotLowerLimit.in(Radians),
    //   ClimbertakeConstants.Pivot.kPivotUpperLimit.in(Radians)
    // );
    double setpointAngleRad = setpoint.in(Radians);

    double resultVolts = WristConstants.kWristPIDController.calculate(currentAngleRad, setpointAngleRad);

    // ! CHECK APPLIED VOLTAGE IN THE DASHBOARD FIRST BEFORE POWERING THE MOTOR
    if (encoder.isConnected()) motor.setVoltage(resultVolts);
    else motor.setVoltage(0);

    SmartDashboard.putNumber("Wrist/SetpointAngle", Math.toDegrees(setpointAngleRad));
    SmartDashboard.putNumber("Wrist/CurrentAngle", Math.toDegrees(currentAngleRad));
    SmartDashboard.putNumber("Wrist/OutputVolts", resultVolts);
  }
}