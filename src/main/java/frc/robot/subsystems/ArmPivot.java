package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Radians;
import com.revrobotics.spark.SparkMax;
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
import frc.robot.Constants.ArmPivotConstants;

public class ArmPivot extends SubsystemBase {
  // Motor
  private final SparkMax motor;

  // Config
  private final SparkMaxConfig motorConfig;

  // Gyro for angle
  private final Pigeon2 gyro;
  private final StatusSignal<Double> gVecX;
  private final StatusSignal<Double> gVecY;
  private final StatusSignal<Double> gVecZ;

  // Setpoint angle
  private Angle setpoint;

  // Alerts
  private final Alert alert_motorUnreachable = new Alert(getName() + " motor unreachable", AlertType.kError);
  private final Alert alert_gyroUnreachable = new Alert(getName() + " gyro unreachable", AlertType.kError);

  // Device check notifier
  private final Notifier deviceCheckNotifier = new Notifier(this::deviceCheck);

  /** Creates a new ArmPivot. */
  public ArmPivot() {
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

    // Apply motor config
    motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    // Create and configure gyro
    this.gyro = new Pigeon2(ArmPivotConstants.kArmPivotGyroID);
    this.gVecX = gyro.getGravityVectorX();
    this.gVecY = gyro.getGravityVectorY();
    this.gVecZ = gyro.getGravityVectorZ();
    
    // Setpoint angle
    this.setpoint = getAngle();

    // Device check
    deviceCheckNotifier.startPeriodic(10);
  }

  public void deviceCheck() {
    try {
      motor.getFirmwareVersion();
      alert_motorUnreachable.set(false);
    } catch (Exception e) {
      alert_motorUnreachable.set(true);
      DriverStation.reportError(alert_motorUnreachable.getText(), false);
    }

    if (gyro.isConnected()) {
      alert_gyroUnreachable.set(false);
    } else {
      alert_gyroUnreachable.set(true);
      DriverStation.reportError(alert_gyroUnreachable.getText(), false);
    }
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
    return getGyroAngle();
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
    // ! CLAMPS ANGLE HERE
    // double setpointAngleRad = MathUtil.clamp(
    //   setpoint.in(Radians),
    //   ClimbertakeConstants.Pivot.kPivotLowerLimit.in(Radians),
    //   ClimbertakeConstants.Pivot.kPivotUpperLimit.in(Radians)
    // );
    double setpointAngleRad = setpoint.in(Radians);

    double resultVolts = ArmPivotConstants.kArmPivotPIDController.calculate(currentAngleRad, setpointAngleRad);

    // ! CHECK APPLIED VOLTAGE IN THE DASHBOARD FIRST BEFORE POWERING THE MOTOR
    // pivotMotor.setVoltage(resultVolts);

    SmartDashboard.putNumber("ArmPivot/SetpointAngle", Math.toDegrees(setpointAngleRad));
    SmartDashboard.putNumber("ArmPivot/CurrentAngle", Math.toDegrees(currentAngleRad));
    SmartDashboard.putNumber("ArmPivot/OutputVolts", resultVolts);
  }
}