package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
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
import frc.robot.Constants.ClimbertakeConstants;

public class ClimbertakePivot extends SubsystemBase {
  // * Motor
  private final SparkMaxConfig pivotConfig;
  private final SparkMax pivotMotor;

  // * Encoder
  private final DutyCycleEncoder pivotEncoder;

  // * Setpoint
  private Angle setpoint;

  // * Alerts
  private final Alert alert_motorUnreachable = new Alert(getName() + " motor unreachable", AlertType.kError);
  private final Alert alert_encoderUnreachable = new Alert(getName() + " encoder unreachable", AlertType.kError);

  // Device check notifier
  private final Notifier deviceCheckNotifier = new Notifier(this::deviceCheck);

  public ClimbertakePivot() {
    // * Create and configure the Motor
    // Motor
    pivotMotor = new SparkMax(ClimbertakeConstants.Pivot.kPivotMotorID, MotorType.kBrushless);

    // Config
    pivotConfig = new SparkMaxConfig();
    pivotConfig
      .idleMode(IdleMode.kBrake)
      .openLoopRampRate(ClimbertakeConstants.Pivot.kPivotMotorRampRate)
      .closedLoopRampRate(ClimbertakeConstants.Pivot.kPivotMotorRampRate)
      .smartCurrentLimit(ClimbertakeConstants.Pivot.kPivotMotorCurrentLimit)
      .voltageCompensation(12);

    // Apply config
    pivotMotor.configure(pivotConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    // Configure PID
    ClimbertakeConstants.Pivot.kPivotPIDController.setTolerance(ClimbertakeConstants.Pivot.epsilon.in(Radians));
    ClimbertakeConstants.Pivot.kPivotPIDController.disableContinuousInput();

    // * Create and configure the Encoder
    pivotEncoder = new DutyCycleEncoder(ClimbertakeConstants.Pivot.kPivotEncoderPort);

    SmartDashboard.putData("ClimbertakePivot/PID", ClimbertakeConstants.Pivot.kPivotPIDController);

    // * Setpoint
    setpoint = getAngle();

    // * Device check
    deviceCheckNotifier.startPeriodic(10);
  }

  private void deviceCheck() {
    try {
      pivotMotor.getFirmwareVersion();
      alert_motorUnreachable.set(false);
    } catch (Exception e) {
      alert_motorUnreachable.set(true);
      DriverStation.reportError(getName() + " encoder unreachable", false);
    }

    if (pivotEncoder.isConnected()) {
      alert_encoderUnreachable.set(false);
    } else {
      alert_encoderUnreachable.set(true);
      DriverStation.reportError(getName() + " piece sensor unreachable", false);
    }
  }

  /**
   * Returns the Angle with the applied offset
   * @return
   */
  public Angle getAngle() {
    return Rotations.of(pivotEncoder.get()).plus(ClimbertakeConstants.Pivot.kPivotEncoderOffset);
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
    this.setpoint = setpoint;
  }

  public void stop() {
    this.pivotMotor.setVoltage(0);
  }

  /**
   * Sets the setpoint of the PID controller
   * @param setpoint
   * @return
   */
  public Command setSetpointCommand(Angle setpoint) {
    return runOnce(() -> setSetpoint(setpoint));
  }

  public Command setVoltageCommand(double voltage) {
    return runOnce(() -> pivotMotor.setVoltage(voltage));
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
    double currentAngleRad = getAngle().in(Radians);
    // ! CLAMPS ANGLE HERE
    double setpointAngleRad = MathUtil.clamp(
      setpoint.in(Radians),
      ClimbertakeConstants.Pivot.kPivotLowerLimit.in(Radians),
      ClimbertakeConstants.Pivot.kPivotUpperLimit.in(Radians)
    );

    double pidOutputVolts = ClimbertakeConstants.Pivot.kPivotPIDController.calculate(currentAngleRad, setpointAngleRad);
    double feedforwardVolts = ClimbertakeConstants.Pivot.kPivotFeedforward.calculate(currentAngleRad, pidOutputVolts);
    double resultVolts = pidOutputVolts;

    // ! CHECK APPLIED VOLTAGE IN THE DASHBOARD FIRST BEFORE POWERING THE MOTOR
    if (pivotEncoder.isConnected()) {
      //pivotMotor.setVoltage(resultVolts);
    } else {
      this.stop();
    }

    // Logging
    SmartDashboard.putNumber("Climbertake/Pivot/SetpointAngle", Math.toDegrees(setpointAngleRad));
    SmartDashboard.putNumber("Climbertake/Pivot/CurrentAngle", Math.toDegrees(currentAngleRad));
    SmartDashboard.putNumber("Climbertake/Pivot/OutputVolts", resultVolts);
  }
}
