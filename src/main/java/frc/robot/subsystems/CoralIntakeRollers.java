package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.reduxrobotics.sensors.canandcolor.Canandcolor;
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
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Constants.CoralIntakeRollerConstants;

public class CoralIntakeRollers extends SubsystemBase {
  // * Motor
  private final SparkMax motor;
  private final SparkMaxConfig motorConfig;

  // * Piece sensor
  private final Canandcolor pieceSensor;

  // * Piece detection
  private Debouncer hasPieceDebouncer = new Debouncer(0.15, DebounceType.kRising);
  private boolean hasPiece = false;

  // *Alerts
  private final Alert alert_upperMotorUnreachable = new Alert(getName() + " motor unreachable.", AlertType.kError);
  private final Alert alert_pieceSensorUnreachable = new Alert(getName() + " piece sensor unreachable.", AlertType.kError);

  // * Device check
  private final Notifier deviceCheckNotifier = new Notifier(this::deviceCheck);

  public CoralIntakeRollers() {
    // * Motor
    this.motor = new SparkMax(Constants.CoralIntakeRollerConstants.kUpperMotorId, MotorType.kBrushless);

    // Upper motor config
    this.motorConfig = new SparkMaxConfig();
    this.motorConfig
      .idleMode(IdleMode.kBrake)
      .openLoopRampRate(CoralIntakeRollerConstants.kMotorRampRate)
      .closedLoopRampRate(CoralIntakeRollerConstants.kMotorRampRate)
      .smartCurrentLimit(CoralIntakeRollerConstants.kMotorCurrentLimit)
      .voltageCompensation(12);

    // Configure upper motor
    this.motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    // * Piece sensor
    this.pieceSensor = new Canandcolor(Constants.CoralIntakeRollerConstants.kPieceSensorId);
    this.pieceSensor.setPartyMode(10);
  
    // * Start device check notifier
    deviceCheckNotifier.setName(getName() + " Device Check");
    deviceCheckNotifier.startPeriodic(Constants.deviceCheckPeriod);
  }

  private void deviceCheck() {
    try {
      motor.getFirmwareVersion();
      alert_upperMotorUnreachable.set(false);
    } catch (Exception e) {
      alert_upperMotorUnreachable.set(true);
      DriverStation.reportError(getName() + " motor unreachable", false);
    }

    if (pieceSensor.isConnected()) {
      alert_pieceSensorUnreachable.set(false);
    } else {
      alert_pieceSensorUnreachable.set(true);
      DriverStation.reportError(getName() + " piece sensor unreachable", false);
    }
  }

  public boolean getHasPieceRaw() {
    return 
      pieceSensor.getProximity() < CoralIntakeRollerConstants.kProximityThreshold;
  }

  public boolean getHasPiece() {
    return hasPiece;
  }
  
  /**
   * Sets the rollers to intake
   */
  public void setIn() {
    motor.setVoltage(Constants.CoralIntakeRollerConstants.kRollersInVoltage);
  }

  /**
   * Sets the rollers to outtake
   */
  public void setOut() {
    motor.setVoltage(Constants.CoralIntakeRollerConstants.kRollersOutVoltage);
  }

  /**
   * Stops the rollers
   */
  public void stop() {
    motor.setVoltage(0);
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

  public Command intakeUntilPieceDetected() {
    return new RunCommand(this::setIn, this).until(this::getHasPiece).andThen(stopCommand());
  }
  
  @Override
  public void periodic() {
    boolean hasPieceRaw = this.getHasPieceRaw();
    this.hasPiece = this.hasPieceDebouncer.calculate(hasPieceRaw);

    SmartDashboard.putBoolean(getName() + "/HasPiece", hasPiece);
    SmartDashboard.putBoolean(getName() + "/HasPieceRaw", hasPieceRaw);
  }
}
