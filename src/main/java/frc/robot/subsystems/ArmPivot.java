// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.ArmPivotConstants;

public class ArmPivot extends SubsystemBase {
  private final SparkFlex motor;
  private final SparkFlexConfig motorConfig;
  private final CANcoder encoder;
  private final ProfiledPIDController pidController;

  
  private boolean pidEnabled = true;
  private double goalPosition;

  /** Creates a new ArmPivot. */
  public ArmPivot() {
    this.motor = new SparkFlex(ArmPivotConstants.motorID, MotorType.kBrushless);
    this.motorConfig = new SparkFlexConfig();
    this.encoder = new CANcoder(ArmPivotConstants.encodeID);

    pidController = new ProfiledPIDController(
      ArmPivotConstants.pidConstants.kP,
      ArmPivotConstants.pidConstants.kI,
      ArmPivotConstants.pidConstants.kD,
      ArmPivotConstants.constraints
    );

    motorConfig.inverted(ArmPivotConstants.PhysicalModel.inverted);
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
    pidController.reset(getPosition());
    pidEnabled = true;
  }

  /**
   * Gets the current position of the wrist
   * @return the current position of the wrist in Rotations
   */
  public double getPosition() {
    return encoder.getPosition().getValueAsDouble();
  }

  /**
   * Sets the goal position of the wrist
   * 
   * @param position The position you want to set in Rotations. It is clamped
   */
  public void setGoalPosition(double position) {
    MathUtil.clamp(position, ArmPivotConstants.PhysicalModel.minRot, ArmPivotConstants.PhysicalModel.maxRot);
    
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
    SmartDashboard.putNumber("ArmPivot/position", getPosition());
    SmartDashboard.putNumber("ArmPivot/goalPosition", goalPosition);
    SmartDashboard.putNumber("ArmPivot/motorOutput", motor.get());
    SmartDashboard.putNumber("ArmPivot/pidError", pidController.getPositionError());
    SmartDashboard.putNumber("ArmPivot/pidOutput", pidController.calculate(getPosition()));
  }

}
