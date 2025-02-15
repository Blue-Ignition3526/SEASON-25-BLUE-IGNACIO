// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems;

import com.reduxrobotics.sensors.canandcolor.Canandcolor;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeCoral extends SubsystemBase {
  /** Creates a new IntakeCoral. */

  //motors 
  private final SparkMax m_upperMotor;
  private final SparkMaxConfig m_upperMotorConfig;
  private final SparkMax m_lowerMotor;
  private final SparkMaxConfig m_lowerMotorConfig;

  //closed loop controller 
  private final SparkClosedLoopController rollersClosedLoopController;  

  // sensor 
  private final Canandcolor pieceSensor;

  // Alerts
  private final Alert alert_motorUnreachable = new Alert(getName() + " motor unreachable", AlertType.kError);
  private final Alert alert_pieceSensorUnreachable = new Alert(getName() + " piece sensor unreachable", AlertType.kError);

  // Device check notifier
  private final Notifier deviceCheckNotifier = new Notifier(this::deviceCheck);


  public IntakeCoral() {
    
    //upper motor config
    this.m_upperMotor = new SparkMax(Constants.IntakeCoral.kUpperMotorId, MotorType.kBrushless);
    this.m_upperMotorConfig = new SparkMaxConfig();
    this.m_upperMotorConfig.idleMode(IdleMode.kBrake);
    this.m_upperMotor.configure(m_upperMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    //lower motor config 
    this.m_lowerMotor = new SparkMax(Constants.IntakeCoral.kLowerMotorId, MotorType.kBrushless);
    this.m_lowerMotorConfig = new SparkMaxConfig();
    this.m_lowerMotorConfig.idleMode(IdleMode.kBrake);
    this.m_lowerMotorConfig.follow(m_upperMotor, true);

    // Create closed loop controller
    this.rollersClosedLoopController = m_upperMotor.getClosedLoopController();

    // Create piece sensor
    pieceSensor = new Canandcolor(Constants.IntakeCoral.kSensorId);
    pieceSensor.setLampLEDBrightness(Constants.IntakeCoral.kPieceSensorLedBrightness);
  
    // Start device check notifier
    deviceCheckNotifier.startPeriodic(10);

  }

  private void deviceCheck() {
    try {
      m_upperMotor.getFirmwareVersion();
      alert_motorUnreachable.set(false);
    } catch (Exception e) {
      alert_motorUnreachable.set(true);
      DriverStation.reportError(getName() + " motor unreachable", false);
    }

    try {
      m_lowerMotor.getFirmwareVersion();
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
    m_upperMotor.setVoltage(Constants.IntakeCoral.kRollersInVoltage);
  }

  /**
   * Sets the rollers to outtake
   */
  public void setOut() {
    m_upperMotor.setVoltage(Constants.IntakeCoral.kRollersOutVoltage);
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
      Math.abs(pieceSensor.getRed() - Constants.IntakeCoral.kCoralColor.red) < Constants.IntakeCoral.kCoralColorThreshold &&
      Math.abs(pieceSensor.getGreen() - Constants.IntakeCoral.kCoralColor.green) < Constants.IntakeCoral.kCoralColorThreshold &&
      Math.abs(pieceSensor.getBlue() - Constants.IntakeCoral.kCoralColor.blue) < Constants.IntakeCoral.kCoralColorThreshold
    );

    boolean proximityTriggered = pieceSensor.getProximity() > Constants.IntakeCoral.kCoralProximityThreshold;

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
    SmartDashboard.putString(getName() + "/Color", getColor().toHexString());
  }

}
