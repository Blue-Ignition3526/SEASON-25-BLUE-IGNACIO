package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
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

  // Encoder
  private final RelativeEncoder encoder;

  // Setpoint angle
  private Angle setpoint;

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

    this.motorConfig.encoder
      .positionConversionFactor(1. / 240.);
      
    // Get and configure encoder
    this.encoder = motor.getEncoder();
    
    // Apply motor config
    motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    // Setpoint angle
    this.setpoint = getAngle();

    SmartDashboard.putData("ArmPivot/PID", ArmPivotConstants.kArmPivotPIDController);
  }

  /*
   * Resets the angle
   */
  public void resetAngle() {
    encoder.setPosition(0);
  }

  public Command resetAngleCommand() {
    return runOnce(this::resetAngle);
  }

  /**
   * Gets the current angle of the ArmPivot
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

    // ! CHECK APPLIED VOLTAGE IN THE DASHBOARD FIRST BEFORE POWERING THE MOTOR
    motor.setVoltage(resultVolts);

    SmartDashboard.putNumber("ArmPivot/SetpointAngle", Math.toDegrees(setpointAngleRad));
    SmartDashboard.putNumber("ArmPivot/CurrentAngle", Math.toDegrees(currentAngleRad));
    SmartDashboard.putNumber("ArmPivot/OutputVolts", resultVolts);
  }
}