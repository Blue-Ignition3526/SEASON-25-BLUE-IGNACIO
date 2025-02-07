// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.WristConstants;

public class Wrist extends SubsystemBase {
  private final SparkFlex motor;
  private final SparkFlexConfig motorConfig;
  private final DutyCycleEncoder encoder;
  private final ProfiledPIDController pidController;

  
  private boolean pidEnabled = true;
  private double goalPosition;

  /** Creates a new Wrist. */
  public Wrist() {
    this.motor = new SparkFlex(WristConstants.motorID, MotorType.kBrushless);
    this.motorConfig = new SparkFlexConfig();
    this.encoder = new DutyCycleEncoder(WristConstants.encodePort);

    pidController = new ProfiledPIDController(
      WristConstants.pidConstants.kP,
      WristConstants.pidConstants.kI,
      WristConstants.pidConstants.kD,
      WristConstants.constraints
    );

    motorConfig.inverted(WristConstants.PhysicalModel.inverted);
  }

  /**
   * Stops the wrist, disables PID
   */
  public void stop() {
    pidEnabled = false;
    motor.set(0);
  }

  /**
   * Enables PID
   */
  public void restart() {
    pidEnabled = true;
  }

  /**
   * Gets the current position of the wrist
   * @return the current position of the wrist in Rotations
   */
  public double getPosition() {
    return encoder.get();
  }

  /**
   * Sets the goal position of the wrist
   * 
   * @param position The position you want to set in Rotations. It is clamped
   */
  public void setGoalPosition(double position) {
    MathUtil.clamp(position, WristConstants.PhysicalModel.minRot, WristConstants.PhysicalModel.maxRot);
    
    this.goalPosition = position;
    pidController.setGoal(position);
  }

  /**
   * Gets the goal position of the wrist
   * @return the goal position of the wrist in Rotations
   */
  public double getGoalPosition() {
    return goalPosition;
  }

  public boolean isAtGoal() {
    return pidController.atGoal();
  }

  @Override
  public void periodic() {
    if(pidEnabled) motor.set(pidController.calculate(getPosition()));

    log();
  }

  public Command goTo(double angle) {
    return new FunctionalCommand(
      // Init
      ()->setGoalPosition(angle),
      // Periodic
      () -> {},
      // End
      interrupted -> {},
      // isfinished
      () -> isAtGoal(),
      // addRequirements
      this);
  }

  public void log() {
    SmartDashboard.putNumber("Wrist/position", getPosition());
    SmartDashboard.putNumber("Wrist/goalPosition", goalPosition);
    SmartDashboard.putNumber("Wrist/motorOutput", motor.get());
    SmartDashboard.putNumber("Wrist/encoderPosition", encoder.get());
    SmartDashboard.putNumber("Wrist/pidError", pidController.getPositionError());
    SmartDashboard.putNumber("Wrist/pidOutput", pidController.calculate(getPosition()));
  }
}