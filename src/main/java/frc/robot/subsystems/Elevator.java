package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// TODO: Check unused stuff
public class Elevator extends SubsystemBase {
  // * Setpoints
  public static enum ElevatorPosition {
    L1(118),
    L2(192),
    L3(324),
    L4(600),
    HOME(0.0),
    SOURCE(118);

    private double position;
    
    private ElevatorPosition(double position) {
      this.position = position;
    }

    public double getPosition() {
      return position;
    }
  }

  // * Left motor
  private final SparkFlex leftElevatorMotor;
  private final SparkFlexConfig leftElevatorMotorConfig;

  // * Right motor
  private final SparkFlex rightElevatorMotor;
  private final SparkFlexConfig rightElevatorMotorConfig;

  // * Encoder (right motor integrated encoder)
  private final RelativeEncoder encoder;

  // * Status
  private double setpoint;
  private ElevatorPosition setpointEnum = null;
  private boolean pidEnabled = false;

  // * Alerts
  private Alert alert_leftMotorUnreachable = new Alert(getName() + " left motor unreachable.", AlertType.kError);
  private Alert alert_rightMotorUnreachable = new Alert(getName() + " right motor unreachable.", AlertType.kError);
  private Alert alert_encoderUnreachable = new Alert(getName() + " encoder unreachable.", AlertType.kError);
  private Alert alert_pidDisabled = new Alert(getName() + " PID disabled.", AlertType.kInfo);

  // * Device check
  private Notifier deviceCheckNotifier = new Notifier(this::deviceCheck);

  public Elevator() {
    // * Right motor (MASTER)
    this.rightElevatorMotor = new SparkFlex(ElevatorConstants.kRightMotorID, MotorType.kBrushless);

    // Configure motor
    this.rightElevatorMotorConfig = new SparkFlexConfig();
    this.rightElevatorMotorConfig
      .voltageCompensation(12)
      .idleMode(IdleMode.kBrake)
      .inverted(true)
      .smartCurrentLimit(ElevatorConstants.kElevatorMotorCurrentLimit)
      .openLoopRampRate(ElevatorConstants.kElevatorMotorRampRate)
      .closedLoopRampRate(ElevatorConstants.kElevatorMotorRampRate);

    // Apply right motor configuration
    this.rightElevatorMotor.configure(rightElevatorMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    // * Left motor (FOLLOWER)
    this.leftElevatorMotor = new SparkFlex(ElevatorConstants.kLeftMotorID, MotorType.kBrushless);

    // Configure motor
    this.leftElevatorMotorConfig = new SparkFlexConfig();
    this.leftElevatorMotorConfig
      .voltageCompensation(12)
      .idleMode(IdleMode.kBrake)
      .smartCurrentLimit(ElevatorConstants.kElevatorMotorCurrentLimit)
      .openLoopRampRate(ElevatorConstants.kElevatorMotorRampRate)
      .closedLoopRampRate(ElevatorConstants.kElevatorMotorRampRate)
      .follow(rightElevatorMotor, true);

    // Apply left motor configuration
    this.leftElevatorMotor.configure(leftElevatorMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    // * Encoder
    this.encoder = rightElevatorMotor.getEncoder();

    // * Set setpoint to initial postiion
    this.setpoint = getPosition();

    // * Elevator PID for tuning
    SmartDashboard.putData("Elevator/PID", ElevatorConstants.kElevatorPIDController);

    // Start device check
    deviceCheckNotifier.startPeriodic(Constants.deviceCheckPeriod);
  }

  private void deviceCheck() {
    try {
      rightElevatorMotor.getFirmwareVersion();
      alert_rightMotorUnreachable.set(false);
    } catch (Exception e) {
      alert_rightMotorUnreachable.set(true);
      DriverStation.reportError(alert_rightMotorUnreachable.getText(), false);
    }

    try {
      leftElevatorMotor.getFirmwareVersion();
      alert_leftMotorUnreachable.set(false);
    } catch (Exception e) {
      alert_leftMotorUnreachable.set(true);
      DriverStation.reportError(alert_leftMotorUnreachable.getText(), false);
    }

    try {
      encoder.getPosition();
      alert_encoderUnreachable.set(false);
    } catch (Exception e) {
      alert_encoderUnreachable.set(true);
      DriverStation.reportError(alert_encoderUnreachable.getText(), false);
    }
  }

  /**
   * Get the current position of the elevator
   * @return
   */
  public double getPosition() {
    return this.encoder.getPosition();
  }

  /**
   * Set the setpoint of the elevator
   * @param setpoint
   */
  public void setSetpoint(double setpoint) {
    this.setpoint = setpoint;
    this.setpointEnum = null;
    this.pidEnabled = true;
  }

    /**
   * Set the setpoint of the elevator
   * @param setpoint
   */
  public void setSetpoint(ElevatorPosition setpoint) {
    this.setpoint = setpoint.getPosition();
    this.setpointEnum = setpoint;
    this.pidEnabled = true;
  }

  /**
   * Stop the elevator
   */
  public void stop() {
    this.pidEnabled = false;
    this.rightElevatorMotor.setVoltage(0);
  }

  public void setVoltage(double voltage) { 
    this.pidEnabled = false;
    rightElevatorMotor.setVoltage(voltage);
  }

  public Command setVoltageCommand(double voltage) {
    return runOnce(() -> setVoltage(voltage));
  }

  public Command stopCommand() {
    return runOnce(this::stop);
  }

  public Command setSetpointCommand(double setpoint){
    return runOnce(() -> setSetpoint(setpoint));
  }

  public Command setSetpointCommand(ElevatorPosition setpoint){
    return runOnce(() -> setSetpoint(setpoint));
  }

  public Command resetElevatorPositionCommand() {
    return runOnce(() -> encoder.setPosition(0));
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    double currentPosition = getPosition();

    // Calculate needed voltage
    double pidOutputVolts = ElevatorConstants.kElevatorPIDController.calculate(currentPosition, setpoint);
    double resultVolts = pidOutputVolts;
    
    // Set the voltage to the motor
    // ! CHECK APPLIED VOLTAGE IN THE DASHBOARD FIRST BEFORE POWERING THE MOTOR
    if(pidEnabled) rightElevatorMotor.setVoltage(resultVolts);
    
    // Telemetry 
    SmartDashboard.putNumber("Elevator/AppliedOutput", rightElevatorMotor.get());
    SmartDashboard.putNumber("Elevator/CurrentPosition", currentPosition);
    SmartDashboard.putNumber("Elevator/SetpointPosition", setpoint);
    SmartDashboard.putNumber("Elevator/SetpointVoltage", resultVolts);
    SmartDashboard.putNumber("Elevator/LeftCurrent", leftElevatorMotor.getOutputCurrent());
    SmartDashboard.putNumber("Elevator/RightCurrent", rightElevatorMotor.getOutputCurrent());
  }
}
