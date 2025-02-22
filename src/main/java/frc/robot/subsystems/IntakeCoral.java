package frc.robot.subsystems;

import com.reduxrobotics.sensors.canandcolor.Canandcolor;
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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.IntakeCoralConstants;

public class IntakeCoral extends SubsystemBase {
  // Upper motor
  private final SparkMax m_upperMotor;
  private final SparkMaxConfig m_upperMotorConfig;

  // Lower motor
  private final SparkMax m_lowerMotor;
  private final SparkMaxConfig m_lowerMotorConfig;

  // Piece sensor 
  private final Canandcolor pieceSensor;

  // Alternate piece detection
  private Debouncer currentPieceDetectionDebouncer = new Debouncer(0.25, DebounceType.kRising);
  private boolean currentPieceDetectionState = false;

  // Alerts
  private final Alert alert_upperMotorUnreachable = new Alert(getName() + " motor unreachable", AlertType.kError);
  private final Alert alert_lowerMotorUnreachable = new Alert(getName() + " motor unreachable", AlertType.kError);
  private final Alert alert_pieceSensorUnreachable = new Alert(getName() + " piece sensor unreachable", AlertType.kError);

  // Device check notifier
  private final Notifier deviceCheckNotifier = new Notifier(this::deviceCheck);

  public IntakeCoral() {
    // Upper motor
    this.m_upperMotor = new SparkMax(Constants.IntakeCoralConstants.kUpperMotorId, MotorType.kBrushless);

    // Upper motor config
    this.m_upperMotorConfig = new SparkMaxConfig();
    this.m_upperMotorConfig
      .idleMode(IdleMode.kBrake)
      .openLoopRampRate(IntakeCoralConstants.kMotorRampRate)
      .closedLoopRampRate(IntakeCoralConstants.kMotorRampRate)
      .smartCurrentLimit(IntakeCoralConstants.kMotorCurrentLimit)
      .voltageCompensation(12);

    // Configure upper motor
    this.m_upperMotor.configure(m_upperMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    // Lower motor 
    this.m_lowerMotor = new SparkMax(Constants.IntakeCoralConstants.kLowerMotorId, MotorType.kBrushless);

    // Lower motor config
    this.m_lowerMotorConfig = new SparkMaxConfig();
    this.m_lowerMotorConfig
      .idleMode(IdleMode.kBrake)
      .openLoopRampRate(IntakeCoralConstants.kMotorRampRate)
      .closedLoopRampRate(IntakeCoralConstants.kMotorRampRate)
      .smartCurrentLimit(IntakeCoralConstants.kMotorCurrentLimit)
      .voltageCompensation(12)
      // * Lower motor follows upper motor inverted
      .follow(m_upperMotor, true);

    // Create piece sensor
    pieceSensor = new Canandcolor(Constants.IntakeCoralConstants.kSensorId);
    pieceSensor.setLampLEDBrightness(Constants.IntakeCoralConstants.kPieceSensorLedBrightness);
  
    // Start device check notifier
    deviceCheckNotifier.startPeriodic(10);
  }

  private void deviceCheck() {
    try {
      m_upperMotor.getFirmwareVersion();
      alert_upperMotorUnreachable.set(false);
    } catch (Exception e) {
      alert_upperMotorUnreachable.set(true);
      DriverStation.reportError(getName() + " motor unreachable", false);
    }

    try {
      m_lowerMotor.getFirmwareVersion();
      alert_lowerMotorUnreachable.set(false);
    } catch (Exception e) {
      alert_lowerMotorUnreachable.set(true);
      DriverStation.reportError(getName() + " motor unreachable", false);
    }    

    if (pieceSensor.isConnected(10)) {
      alert_pieceSensorUnreachable.set(false);
    } else {
      alert_pieceSensorUnreachable.set(true);
      DriverStation.reportError(getName() + " piece sensor unreachable", false);
    }
  }

  
  /**
   * Sets the rollers to intake
   */
  public void setIn() {
    m_upperMotor.setVoltage(Constants.IntakeCoralConstants.kRollersInVoltage);
  }

  /**
   * Sets the rollers to outtake
   */
  public void setOut() {
    m_upperMotor.setVoltage(Constants.IntakeCoralConstants.kRollersOutVoltage);
  }

  /**
   * Stops the rollers
   */
  public void stop() {
    m_upperMotor.setVoltage(0);
  }


  /**
   * Gets the color of the piece sensor
   * 
   * @return the color of the piece sensor
   */
  public Color getColor() {
    return pieceSensor.getColor().toWpilibColor();
  }

  /**
   * Returns if a piece was detected
   * @return
   */
  public boolean hasPiece() {
    boolean colorIsSimilar = (
      Math.abs(pieceSensor.getRed() - Constants.IntakeCoralConstants.kCoralColor.red) < Constants.IntakeCoralConstants.kCoralColorThreshold &&
      Math.abs(pieceSensor.getGreen() - Constants.IntakeCoralConstants.kCoralColor.green) < Constants.IntakeCoralConstants.kCoralColorThreshold &&
      Math.abs(pieceSensor.getBlue() - Constants.IntakeCoralConstants.kCoralColor.blue) < Constants.IntakeCoralConstants.kCoralColorThreshold
    );

    boolean proximityTriggered = pieceSensor.getProximity() > Constants.IntakeCoralConstants.kCoralProximityThreshold;

    return colorIsSimilar || proximityTriggered;
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

  public Command setInUntilHasPieceCommand(){
    return runOnce(()-> setInCommand().until(this::hasPiece).andThen(stopCommand()));
  }

  @Override
  public void periodic() {
    // Update piece detection debouncer
    this.currentPieceDetectionState = currentPieceDetectionDebouncer.calculate(m_upperMotor.getOutputCurrent() > Constants.IntakeCoralConstants.kPieceDetectionCurrent);

    SmartDashboard.putString(getName() + "/Color", getColor().toHexString());
  }
}
