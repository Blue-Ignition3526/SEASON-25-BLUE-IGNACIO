package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;
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
import lib.team3526.utils.BluePWMEncoder;

public class ArmPivot extends SubsystemBase {
  // Motor
  private final SparkMax motor;

  // Config
  private final SparkMaxConfig motorConfig;

  // Through bore encoder
  private final BluePWMEncoder encoder;

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

    // Apply motor config
    motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    // Create and configure encoder
    this.encoder = new BluePWMEncoder(ArmPivotConstants.kArmPivotEncoderPort);
    this.encoder.setOffset(ArmPivotConstants.kArmPivotEncoderOffset.in(Rotations));

    // Setpoint angle
    this.setpoint = getAngle();
  }

  /**
   * Gets the current angle of the ArmPivot
   */
  public Angle getAngle() {
    return encoder.getAngle();
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