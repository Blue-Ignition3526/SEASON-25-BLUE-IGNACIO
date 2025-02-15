package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Radians;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbertakeConstants;
import lib.team3526.utils.BluePWMEncoder;

public class ClimbertakePivot extends SubsystemBase {
  // * Motor
  private final SparkMaxConfig pivotConfig;
  private final SparkMax pivotMotor;

  // * Encoder
  private final BluePWMEncoder pivotEncoder;

  // * Setpoint
  private Angle setpoint;

  public ClimbertakePivot() {
    // * Create and configure the Motor
    // Motor
    pivotMotor = new SparkMax(ClimbertakeConstants.Pivot.kPivotMotorID, MotorType.kBrushless);

    // Config
    pivotConfig = new SparkMaxConfig();
    pivotConfig
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
    pivotEncoder = new BluePWMEncoder(ClimbertakeConstants.Pivot.kPivotEncoderPort);

    // * Setpoint
    setpoint = getAngle();
  }

  /**
   * Returns the Angle with the applied offset
   * @return
   */
  public Angle getAngle() {
    return pivotEncoder.getAngle();
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
    double resultVolts = pidOutputVolts + feedforwardVolts;

    // ! CHECK APPLIED VOLTAGE IN THE DASHBOARD FIRST BEFORE POWERING THE MOTOR
    // pivotMotor.setVoltage(resultVolts);

    SmartDashboard.putNumber("Climbertake/Pivot/SetpointAngle", Math.toDegrees(setpointAngleRad));
    SmartDashboard.putNumber("Climbertake/Pivot/CurrentAngle", Math.toDegrees(currentAngleRad));
    SmartDashboard.putNumber("Climbertake/Pivot/OutputVolts", resultVolts);
  }
}
