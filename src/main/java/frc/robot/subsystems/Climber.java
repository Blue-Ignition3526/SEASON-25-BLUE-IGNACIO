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
  private final SparkFlex motor;
  private final SparkFlexConfig motorConfig;

  public static enum ServoPosition {
    LOCKED(0),
    UNLOCKED(180);

    private final int position;
    private ServoPosition(int position) {
      this.position = position;
    }

    public int getPosition() {
      return position;
    }
  }

  private final Servo servo;
  /** Creates a new Climber. */
  public Climber() {
    this.motor = new SparkFlex(ClimberConstants.kLowerMotorId, MotorType.kBrushless);
    this.servo = new Servo(ClimberConstants.kServoPort);
    this.motorConfig = new SparkFlexConfig(); 
    
    this.motorConfig
      .idleMode(IdleMode.kBrake)
      .smartCurrentLimit(40);

  }

  public void setVoltage(double volts) {
    this.motor.setVoltage(volts);
  }

  public Command setVoltCommand(double volts) {
    return runOnce(() -> setVoltage(volts));
  }

  public Command setServo(ServoPosition pos) {
    return runOnce(() -> servo.setPosition(pos.getPosition()));
  }

  public Command setUpCommand() {
    return runEnd(() -> {
      setVoltage(5);
      servo.setPosition(ServoPosition.UNLOCKED.getPosition());
    }, () -> { 
      setVoltage(0);
      servo.setPosition(ServoPosition.LOCKED.getPosition());
      }
    );
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
