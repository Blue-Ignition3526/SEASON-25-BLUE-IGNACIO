package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ClimbertakeConstants;

public class AlgaeClimbertakePivot extends SubsystemBase {
  // * Setpoints
  public static enum ClimbertakePosition {
    // 0 corelates to horizontal
    // 90 corelates to vertical
    // ! BE MINDFUL OF MECHANICAL LIMITS
    CLIMB_LOW(Degrees.of(0)),
    CLIMB_HIGH(Degrees.of(40)),
    INTAKE(Degrees.of(40)),
    STORE(Degrees.of(60));

    private Angle angle;
    private ClimbertakePosition(Angle angle) {
      this.angle = angle;
    }

    public Angle getAngle() {
      return angle;
    }
  }

  // * Left Motor
  private final SparkFlex leftMotor;
  private final SparkFlexConfig leftMotorConfig;

  // * Right Motor
  private final SparkFlex rightMotor;
  private final SparkFlexConfig rightMotorConfig;

  // * Encoder
  private final DutyCycleEncoder pivotEncoder;
  private final RelativeEncoder motorEncodeer;

  // * Status
  private Angle setpoint;
  private boolean pidEnabled = false;

  // * Alerts
  private final Alert alert_leftMotorUnreachable = new Alert(getName() + " left motor unreachable", AlertType.kError);
  private final Alert alert_rightMotorUnreachable = new Alert(getName() + " right motor unreachable", AlertType.kError);
  private final Alert alert_encoderUnreachable = new Alert(getName() + " encoder unreachable", AlertType.kError);
  private final Alert alert_pidDisabled = new Alert(getName() + " PID disabled.", AlertType.kInfo);

  // * Device check
  private final Notifier deviceCheckNotifier = new Notifier(this::deviceCheck);

  public AlgaeClimbertakePivot() {
    // * Create and configure left motor
    // Motor
    leftMotor = new SparkFlex(ClimbertakeConstants.Pivot.kLeftPivotMotorID, MotorType.kBrushless);

    // Config
    leftMotorConfig = new SparkFlexConfig();
    leftMotorConfig
      .idleMode(IdleMode.kBrake)
      .openLoopRampRate(ClimbertakeConstants.Pivot.kPivotMotorRampRate)
      .closedLoopRampRate(ClimbertakeConstants.Pivot.kPivotMotorRampRate)
      .smartCurrentLimit(ClimbertakeConstants.Pivot.kPivotMotorCurrentLimit)
      .voltageCompensation(12);

    // Apply config
    leftMotor.configure(leftMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    // * Create and configure right motor
    // Motor
    rightMotor = new SparkFlex(ClimbertakeConstants.Pivot.kRightPivotMotorID, MotorType.kBrushless);

    // Config
    rightMotorConfig = new SparkFlexConfig();
    rightMotorConfig
      .idleMode(IdleMode.kBrake)
      .follow(leftMotor, true)
      .openLoopRampRate(ClimbertakeConstants.Pivot.kPivotMotorRampRate)
      .closedLoopRampRate(ClimbertakeConstants.Pivot.kPivotMotorRampRate)
      .smartCurrentLimit(ClimbertakeConstants.Pivot.kPivotMotorCurrentLimit)
      .voltageCompensation(12);

    // Apply config
    rightMotor.configure(rightMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    // Configure PID
    ClimbertakeConstants.Pivot.kPivotPIDController.setTolerance(ClimbertakeConstants.Pivot.epsilon.in(Radians));
    ClimbertakeConstants.Pivot.kPivotPIDController.disableContinuousInput();

    // * Create and configure the Encoder
    pivotEncoder = new DutyCycleEncoder(ClimbertakeConstants.Pivot.kPivotEncoderPort);

    SmartDashboard.putData("ClimbertakePivot/PID", ClimbertakeConstants.Pivot.kPivotPIDController);

    motorEncodeer = leftMotor.getEncoder();

    // * Setpoint
    setpoint = getAngle();

    

    // * Device check
    deviceCheckNotifier.startPeriodic(Constants.deviceCheckPeriod);
  }

  private void deviceCheck() {
    try {
      leftMotor.getFirmwareVersion();
      alert_leftMotorUnreachable.set(false);
    } catch (Exception e) {
      alert_leftMotorUnreachable.set(true);
      DriverStation.reportError(alert_leftMotorUnreachable.getText(), false);
    }

    try {
      rightMotor.getFirmwareVersion();
      alert_rightMotorUnreachable.set(false);
    } catch (Exception e) {
      alert_rightMotorUnreachable.set(true);
      DriverStation.reportError(alert_rightMotorUnreachable.getText(), false);
    }

    if (pivotEncoder.isConnected()) {
      alert_encoderUnreachable.set(false);
    } else {
      alert_encoderUnreachable.set(true);
      DriverStation.reportError(alert_encoderUnreachable.getText(), false);
    }
  }

  /**
   * Resets the motor encoder using the absolute encoder.
   * @Warning The absolute has a 3:1 reduction with the arm pivot, only use if stowed
   */
  public void resetEncoder() {
    motorEncodeer.setPosition(pivotEncoder.get());
  }
  /**
   * Returns the Angle with the applied offset
   * @return
   */
  public Angle getAngle() {
    return Rotations.of(motorEncodeer.getPosition()).div(3).plus(ClimbertakeConstants.Pivot.kPivotEncoderOffset);
  }

  /**
   * Resets the PID Controller
   */
  public void resetPID() {
    ClimbertakeConstants.Pivot.kPivotPIDController.reset(getAngle().in(Radians));
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
    this.setpoint = Rotations.of(MathUtil.clamp(
      setpoint.in(Rotations),
      ClimbertakeConstants.Pivot.kPivotLowerLimit.in(Rotations),
      ClimbertakeConstants.Pivot.kPivotUpperLimit.in(Rotations)
    ));
  }

  public void stop() {
    this.leftMotor.setVoltage(0);
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
   * Set voltage command
   * @param voltage
   * @return
   */
  public Command setVoltageCommand(double voltage) {
    return runOnce(() -> leftMotor.setVoltage(voltage));
  }

  /**
   * Returns wether or not the PID controller is at the setpoint
   * @return
   */
  public boolean atSetpoint() {
    return ClimbertakeConstants.Pivot.kPivotPIDController.atSetpoint();
  }

  @Override
  public void periodic() {
    // Get angles
    double currentAngleRad = getAngle().in(Radians);
    double setpointAngleRad = setpoint.in(Radians);

    // Calculate voltage
    double pidOutputVolts = ClimbertakeConstants.Pivot.kPivotPIDController.calculate(currentAngleRad, setpointAngleRad);
    @SuppressWarnings("unused")
    double feedforwardVolts = ClimbertakeConstants.Pivot.kPivotFeedforward.calculate(currentAngleRad, pidOutputVolts);
    double resultVolts = pidOutputVolts;

    // if (!pivotEncoder.isConnected()) {
    //   if (pidEnabled) leftMotor.setVoltage(0);
    //   pidEnabled = false;
    // } else {
    //   if (pidEnabled) leftMotor.setVoltage(resultVolts);
    // }

    // PID Alert
    alert_pidDisabled.set(!pidEnabled);

    // Logging
    SmartDashboard.putNumber("Climbertake/Pivot/SetpointAngle", Math.toDegrees(setpointAngleRad));
    SmartDashboard.putNumber("Climbertake/Pivot/CurrentAngle", Math.toDegrees(currentAngleRad));
    SmartDashboard.putNumber("Climbertake/Pivot/OutputVolts", resultVolts);
  }
}
