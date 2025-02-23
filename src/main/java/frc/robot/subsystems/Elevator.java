package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Inches;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import frc.robot.Constants.ElevatorConstants;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Creates a new Elevator. */
public class Elevator extends SubsystemBase {
  // Left motor
  private final SparkMax m_leftElevatorMotor;
  private final SparkMaxConfig m_leftElevatorMotorConfig;

  // Right motor
  private final SparkMax m_rightElevatorMotor;
  private final SparkMaxConfig m_rightElevatorMotorConfig;

  // Encoder (right motor integrated encoder)
  private final RelativeEncoder m_encoder;

  // Setpoint
  private Distance m_setpoint;

  // Alerts
  private Alert alert_leftMotorUnreachable = new Alert(getName() + " left motor unreachable.", AlertType.kError);
  private Alert alert_rightMotorUnreachable = new Alert(getName() + " right motor unreachable.", AlertType.kError);
  private Alert alert_encoderUnreachable = new Alert(getName() + " encoder unreachable.", AlertType.kError);

  // Device check
  private Notifier deviceCheckNotifier = new Notifier(this::deviceCheck);

  public Elevator() {
    // * Right motor (MASTER)
    this.m_rightElevatorMotor = new SparkMax(ElevatorConstants.kRightMotorID, MotorType.kBrushless);

    // Configure motor
    this.m_rightElevatorMotorConfig = new SparkMaxConfig();
    this.m_rightElevatorMotorConfig
      .voltageCompensation(12)
      .idleMode(IdleMode.kBrake)
      .smartCurrentLimit(ElevatorConstants.kElevatorMotorCurrentLimit)
      .openLoopRampRate(ElevatorConstants.kElevatorMotorRampRate)
      .closedLoopRampRate(ElevatorConstants.kElevatorMotorRampRate);
    
    // Configure encoder
    this.m_rightElevatorMotorConfig.encoder.positionConversionFactor(ElevatorConstants.kRotationsToInches);
    this.m_rightElevatorMotorConfig.inverted(true);

    // Apply right motor configuration
    this.m_rightElevatorMotor.configure(m_rightElevatorMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    // * Left motor (FOLLOWER)
    this.m_leftElevatorMotor = new SparkMax(ElevatorConstants.kLeftMotorID, MotorType.kBrushless);

    // Configure motor
    this.m_leftElevatorMotorConfig = new SparkMaxConfig();
    this.m_leftElevatorMotorConfig
      .voltageCompensation(12)
      .idleMode(IdleMode.kBrake)
      .smartCurrentLimit(ElevatorConstants.kElevatorMotorCurrentLimit)
      .openLoopRampRate(ElevatorConstants.kElevatorMotorRampRate)
      .closedLoopRampRate(ElevatorConstants.kElevatorMotorRampRate)
      .follow(m_rightElevatorMotor, true);

    // Apply left motor configuration
    this.m_leftElevatorMotor.configure(m_leftElevatorMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    // * Encoder
    this.m_encoder = m_rightElevatorMotor.getEncoder();

    // * Set setpoint to initial postiion
    this.m_setpoint = getPosition();

    // Start device check
    deviceCheckNotifier.startPeriodic(10);
  }

  private void deviceCheck() {
    try {
      m_rightElevatorMotor.getFirmwareVersion();
      alert_rightMotorUnreachable.set(false);
    } catch (Exception e) {
      alert_rightMotorUnreachable.set(true);
      DriverStation.reportError(alert_rightMotorUnreachable.getText(), false);
    }

    try {
      m_leftElevatorMotor.getFirmwareVersion();
      alert_leftMotorUnreachable.set(false);
    } catch (Exception e) {
      alert_leftMotorUnreachable.set(true);
      DriverStation.reportError(alert_leftMotorUnreachable.getText(), false);
    }

    try {
      m_encoder.getPosition();
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
  public Distance getPosition() {
    return Inches.of(this.m_encoder.getPosition());
  }

  /**
   * Set the setpoint of the elevator
   * @param setpoint
   */
  public void setSetpoint(Distance setpoint) {
    this.m_setpoint = setpoint;
  }

  /**
   * Stop the elevator
   */
  public void stop() {
    this.m_rightElevatorMotor.setVoltage(0);
  }

  public Command setVoltageCommand(double voltage) {
    return runOnce(() -> m_rightElevatorMotor.setVoltage(voltage));
  }

  public Command stopCommand() {
    return runOnce(this::stop);
  }

  //command for establishing the setpoint in inches
  public Command setSetpointCommand(Distance setpoint){
    return runOnce(() -> setSetpoint(setpoint));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    double currentPositionInches = getPosition().in(Inches);
    // ! SETPOINT CLAMPED HERE
    double setpointPositionInches = MathUtil.clamp(
      m_setpoint.in(Inches),
      ElevatorConstants.kElevatorMinHeight.in(Inches),
      ElevatorConstants.kElevatorMaxHeight.in(Inches)
    );

    // Calculate needed voltage
    double pidOutputVolts = ElevatorConstants.kElevatorPIDController.calculate(currentPositionInches, setpointPositionInches);
    double feedforwardVolts = ElevatorConstants.kElevatorFeedforward.calculate(currentPositionInches, pidOutputVolts);
    double resultVolts = MathUtil.clamp(pidOutputVolts + feedforwardVolts, -12, 12);
    
    // Set the voltage to the motor
    // ! CHECK APPLIED VOLTAGE IN THE DASHBOARD FIRST BEFORE POWERING THE MOTOR
    //m_rightElevatorMotor.setVoltage(resultVolts);
    
    //telemetry 
    SmartDashboard.putNumber("Elevator/AppliedOutput", m_rightElevatorMotor.get());
    SmartDashboard.putNumber("Elevator/CurrentPosition", currentPositionInches);
    SmartDashboard.putNumber("Elevator/SetpointPosition", setpointPositionInches);
    SmartDashboard.putNumber("Elevator/SetpointVoltage", resultVolts);
    SmartDashboard.putNumber("Elevator/LeftCurrent", m_leftElevatorMotor.getOutputCurrent());
    SmartDashboard.putNumber("Elevator/RightCurrent", m_rightElevatorMotor.getOutputCurrent());
 
  }
}
