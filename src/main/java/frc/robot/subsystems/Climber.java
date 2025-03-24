// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

//TODO: bruh
public class Climber extends SubsystemBase {
  private final SparkFlex upperMotor;
  private final SparkFlex lowerMotor;
  private final SparkFlexConfig upperMotorConfig;
  private final SparkFlexConfig lowerMotorConfig;

  private final Servo servo;
  /** Creates a new Climber. */
  public Climber() {
    this.upperMotor = new SparkFlex(ClimberConstants.kUpperMotorId, MotorType.kBrushless);
    this.lowerMotor = new SparkFlex(ClimberConstants.kLowerMotorId, MotorType.kBrushless);
    this.servo = new Servo(ClimberConstants.kServoPort);
    this.upperMotorConfig = new SparkFlexConfig();
    this.lowerMotorConfig = new SparkFlexConfig(); 

    this.upperMotorConfig
      .idleMode(IdleMode.kCoast)
      .smartCurrentLimit(40);
    
    this.lowerMotorConfig
      .idleMode(IdleMode.kBrake)
      .smartCurrentLimit(40);

  }

  public void highSetVoltage(double volts) {
    this.upperMotor.setVoltage(volts);
  }

  public void lowSetVoltage(double volts) {
    this.lowerMotor.setVoltage(volts);
  }

  public Command setVoltHighCommand(double volts) {
    return runOnce(() -> highSetVoltage(volts));
  }

  public Command setVoltLowCommand(double volts) {
    return runOnce(() -> lowSetVoltage(volts));
  }

  public Command setServo(double speed) {
    return runOnce(() -> servo.set(speed));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
