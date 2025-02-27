package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.IntakeCoralConstants;

// TODO: Current piece detection
public class CoralIntakeRollers extends SubsystemBase {
  // * Upper motor
  private final SparkMax upperMotor;
  private final SparkMaxConfig upperMotorConfig;

  // * Lower motor
  private final SparkMax lowerMotor;
  private final SparkMaxConfig lowerMotorConfig;

  // * Piece detection
  private Debouncer detectionDebouncer = new Debouncer(0.25, DebounceType.kRising);
  private boolean hasPiece = false;

  // *Alerts
  private final Alert alert_upperMotorUnreachable = new Alert(getName() + " motor unreachable", AlertType.kError);
  private final Alert alert_lowerMotorUnreachable = new Alert(getName() + " motor unreachable", AlertType.kError);

  // * Device check
  private final Notifier deviceCheckNotifier = new Notifier(this::deviceCheck);

  public CoralIntakeRollers() {
    // Upper motor
    this.upperMotor = new SparkMax(Constants.IntakeCoralConstants.kUpperMotorId, MotorType.kBrushless);

    // Upper motor config
    this.upperMotorConfig = new SparkMaxConfig();
    this.upperMotorConfig
      .idleMode(IdleMode.kBrake)
      .openLoopRampRate(IntakeCoralConstants.kMotorRampRate)
      .closedLoopRampRate(IntakeCoralConstants.kMotorRampRate)
      .smartCurrentLimit(IntakeCoralConstants.kMotorCurrentLimit)
      .voltageCompensation(12);

    // Configure upper motor
    this.upperMotor.configure(upperMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    // Lower motor 
    this.lowerMotor = new SparkMax(Constants.IntakeCoralConstants.kLowerMotorId, MotorType.kBrushless);

    // Lower motor config
    this.lowerMotorConfig = new SparkMaxConfig();
    this.lowerMotorConfig
      .idleMode(IdleMode.kBrake)
      .openLoopRampRate(IntakeCoralConstants.kMotorRampRate)
      .closedLoopRampRate(IntakeCoralConstants.kMotorRampRate)
      .smartCurrentLimit(IntakeCoralConstants.kMotorCurrentLimit)
      .voltageCompensation(12)
      // * Lower motor follows upper motor inverted
      .follow(upperMotor, false);

    this.lowerMotor.configure(lowerMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
  
    // Start device check notifier
    deviceCheckNotifier.startPeriodic(Constants.deviceCheckPeriod);
  }

  private void deviceCheck() {
    try {
      upperMotor.getFirmwareVersion();
      alert_upperMotorUnreachable.set(false);
    } catch (Exception e) {
      alert_upperMotorUnreachable.set(true);
      DriverStation.reportError(getName() + " motor unreachable", false);
    }

    try {
      lowerMotor.getFirmwareVersion();
      alert_lowerMotorUnreachable.set(false);
    } catch (Exception e) {
      alert_lowerMotorUnreachable.set(true);
      DriverStation.reportError(getName() + " motor unreachable", false);
    }
  }

  
  /**
   * Sets the rollers to intake
   */
  public void setIn() {
    upperMotor.setVoltage(Constants.IntakeCoralConstants.kRollersInVoltage);
  }

  /**
   * Sets the rollers to outtake
   */
  public void setOut() {
    upperMotor.setVoltage(Constants.IntakeCoralConstants.kRollersOutVoltage);
  }

  /**
   * Stops the rollers
   */
  public void stop() {
    upperMotor.setVoltage(0);
  }

  /**
   * Set the intake to in
   * @return
   */
  public Command setInCommand() {
    return runOnce(this::setIn);
  }

  /**
   * Set the intake to out
   * @return
   */
  public Command setOutCommand() {
    return runOnce(this::setOut);
  }

  /**
   * Stop the intake
   * @return
   */
  public Command stopCommand() {
    return runOnce(this::stop);
  }

  @Override
  public void periodic() {
    double upperCurrent = upperMotor.getOutputCurrent();
    double lowerCurrent = lowerMotor.getOutputCurrent();
    double avgCurrent = (upperCurrent + lowerCurrent) * 0.5;

    // Update piece detection debouncer
    this.hasPiece = detectionDebouncer.calculate(upperMotor.getOutputCurrent() > Constants.IntakeCoralConstants.kPieceDetectionCurrent);

    SmartDashboard.putNumber(getName() + "/UpperCurrent", upperCurrent);
    SmartDashboard.putNumber(getName() + "/LowerCurrent", lowerCurrent);
    SmartDashboard.putNumber(getName() + "/AvgCurrent", avgCurrent);
  }
}
