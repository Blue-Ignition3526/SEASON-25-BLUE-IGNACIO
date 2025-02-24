package frc.robot.subsystems;

import com.reduxrobotics.sensors.canandcolor.Canandcolor;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbertakeConstants;

public class ClimbertakeRollers extends SubsystemBase {
  // Motor
  private final SparkFlex rollers;

  // Closed loop controller
  private final SparkClosedLoopController rollersClosedLoopController;

  // Config
  private final SparkFlexConfig config;

  // Canandcolor piece sensor
  private final Canandcolor pieceSensor;

  // Alerts
  private final Alert alert_motorUnreachable = new Alert(getName() + " motor unreachable", AlertType.kError);
  private final Alert alert_pieceSensorUnreachable = new Alert(getName() + " piece sensor unreachable", AlertType.kError);

  // Device check notifier
  private final Notifier deviceCheckNotifier = new Notifier(this::deviceCheck);

  public ClimbertakeRollers() {
    // Create roller motor
    rollers = new SparkFlex(ClimbertakeConstants.Rollers.kRollersMotorID, MotorType.kBrushless);

    // Create closed loop controller
    rollersClosedLoopController = rollers.getClosedLoopController();

    // Configure roller motor
    config = new SparkFlexConfig();
    config
      .idleMode(IdleMode.kBrake)
      .openLoopRampRate(ClimbertakeConstants.Rollers.kRollersMotorRampRate)
      .closedLoopRampRate(ClimbertakeConstants.Rollers.kRollersMotorRampRate)
      .smartCurrentLimit(ClimbertakeConstants.Rollers.kRollersMotorCurrentLimit)
      .inverted(true)
      .voltageCompensation(12);

    // Apply config
    rollers.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    // Create piece sensor
    pieceSensor = new Canandcolor(ClimbertakeConstants.Rollers.kPieceSensorID);
    pieceSensor.setLampLEDBrightness(ClimbertakeConstants.Rollers.kPieceSensorLedBrightness);

    // Start device check notifier
    deviceCheckNotifier.startPeriodic(10);
  }

  private void deviceCheck() {
    try {
      rollers.getFirmwareVersion();
      alert_motorUnreachable.set(false);
    } catch (Exception e) {
      alert_motorUnreachable.set(true);
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
    rollers.setVoltage(ClimbertakeConstants.Rollers.kRollersInVoltage);
  }

  /**
   * Sets the rollers to outtake
   */
  public void setOut() {
    rollers.setVoltage(ClimbertakeConstants.Rollers.kRollersOutVoltage);
  }

  /**
   * Stops the rollers
   */
  public void stop() {
    rollers.setVoltage(0);
  }

  /**
   * Sets the rollers to store current
   */
  // ! CAREFUL WITH THIS METHOD, IT CAN DAMAGE THE MECHANISM IF UNLOADED
  public void setStore() {
    rollersClosedLoopController.setReference(ClimbertakeConstants.Rollers.kRollersStoreCurrent, ControlType.kCurrent);
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
      Math.abs(pieceSensor.getRed() - ClimbertakeConstants.Rollers.kAlgaeColor.red) < ClimbertakeConstants.Rollers.kAlgaeColorThreshold &&
      Math.abs(pieceSensor.getGreen() - ClimbertakeConstants.Rollers.kAlgaeColor.green) < ClimbertakeConstants.Rollers.kAlgaeColorThreshold &&
      Math.abs(pieceSensor.getBlue() - ClimbertakeConstants.Rollers.kAlgaeColor.blue) < ClimbertakeConstants.Rollers.kAlgaeColorThreshold
    );

    boolean proximityTriggered = pieceSensor.getProximity() > ClimbertakeConstants.Rollers.kAlgaeProximityThreshold;

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

  @Override
  public void periodic() {
    SmartDashboard.putString(getName() + "/Color", getColor().toHexString());
  }
}
