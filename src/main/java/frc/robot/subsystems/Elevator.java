// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.trajectory.ExponentialProfile.Constraints;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {
  /** Creates a new Elevator. */

  //motors
  private final SparkFlex m_rightElevatorMotor;
  private final SparkFlexConfig m_rightElevatorMotorConfig;
  private final SparkFlex m_leftElevatorMotor;
  private final SparkFlexConfig m_leftElevatorMotorConfig;

  //encoder
  private final RelativeEncoder m_encoder;

  //setpoint
  private double m_setpoint = 0.0;

  public Elevator() {
    //motors
    this.m_rightElevatorMotor = new SparkFlex(Constants.Elevator.kRightMotorID, MotorType.kBrushless);
    this.m_rightElevatorMotorConfig = new SparkFlexConfig();
    this.m_rightElevatorMotorConfig.idleMode(IdleMode.kBrake);
    this.m_rightElevatorMotor.configure(m_rightElevatorMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    this.m_leftElevatorMotor = new SparkFlex(Constants.Elevator.kLeftMotorID, MotorType.kBrushless);
    this.m_leftElevatorMotorConfig = new SparkFlexConfig();
    this.m_leftElevatorMotorConfig.idleMode(IdleMode.kBrake);
    this.m_leftElevatorMotorConfig.follow(m_rightElevatorMotor);
    this.m_leftElevatorMotor.configure(m_leftElevatorMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    //encoders
    this.m_encoder = m_rightElevatorMotor.getEncoder();
  }

  //get position of elevator

  public double getPosition(){
    return this.m_encoder.getPosition();
  }

  //get setpoint
  public double getSetpoint(){
    return this.m_setpoint;
  }

  //place the setpoint for elevator

  public void placeSetpoint(double setpoint){
    this.m_setpoint = setpoint;
  }

  //get the absolut difference of the setpoint and current position
  public double getSetpointError(){
    return Math.abs(m_setpoint)-(getPosition());
  }

  // return if elevator is at setpoint
  public boolean atSetpoint(){
    return Math.abs(getSetpointError()) < Constants.Elevator.kElevatorTolerance;
  }

  //stop elevator
  public void stop() {
    this.m_rightElevatorMotor.set(0);
  }

  //command for establishing the setpoint
  public Command setPositionCommand(double position){
    return runOnce(() -> m_setpoint = position);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    double currentPosition = getPosition();
    double desiredPosition = m_setpoint;

    //set Voltage

    double setVoltage = 0;

    // if elevator is not at the deserid point put the voltage to it with pid
    if (!atSetpoint()){
      double pid = Constants.Elevator.kElevatorPIDController.calculate(currentPosition, desiredPosition);
      setVoltage = pid;
    } else {
      setVoltage = 0;
    }
    
    //set voltage to right motor, left should follow
    m_rightElevatorMotor.setVoltage(setVoltage);
    
    //telemetry 
    SmartDashboard.putNumber("Elevator/CurrentPosition", getPosition());
    SmartDashboard.putNumber("Elevator/DesiredPosition", desiredPosition);
    SmartDashboard.putNumber("Elevator/SetVoltage", setVoltage);
    SmartDashboard.putNumber("Elevator/SetpointError", getSetpointError());
    SmartDashboard.putBoolean("Elevator/AtSetpoint", atSetpoint());
    SmartDashboard.putNumber("Elevator/LeftAmperage", m_leftElevatorMotor.getOutputCurrent());
    SmartDashboard.putNumber("Elevator/RightAmperage", m_rightElevatorMotor.getOutputCurrent());
 
  }
}
