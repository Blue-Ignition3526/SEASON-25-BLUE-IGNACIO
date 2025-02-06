// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {
  /** Creates a new Elevator. */

  //motors
  private final SparkMax m_rightElevatorMotor;
  private final SparkMaxConfig m_rightElevatorMotorConfig;
  private final SparkMax m_leftElevatorMotor;
  private final SparkMaxConfig m_leftElevatorMotorConfig;

  //encoder
  private final RelativeEncoder m_encoder;

  //setpoint
  private double m_setpoint = 0.0;

  public Elevator() {
    //motors
    this.m_rightElevatorMotor = new SparkMax(Constants.Elevator.kRightMotorID, MotorType.kBrushless);
    this.m_rightElevatorMotorConfig = new SparkMaxConfig();
    this.m_rightElevatorMotorConfig.idleMode(IdleMode.kBrake);
    this.m_rightElevatorMotorConfig.encoder.positionConversionFactor(m_setpoint);
    this.m_rightElevatorMotor.configure(m_rightElevatorMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    this.m_leftElevatorMotor = new SparkMax(Constants.Elevator.kLeftMotorID, MotorType.kBrushless);
    this.m_leftElevatorMotorConfig = new SparkMaxConfig();
    this.m_leftElevatorMotorConfig.idleMode(IdleMode.kBrake);
    this.m_leftElevatorMotorConfig.follow(m_rightElevatorMotor, true);
    this.m_leftElevatorMotor.configure(m_leftElevatorMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    //encoders
    this.m_encoder = m_rightElevatorMotor.getEncoder();    
  }

  //get position of elevator

  public double getPosition(){
    return this.m_encoder.getPosition() * (Constants.Elevator.kElevatorPulleyDiameter * Math.PI);
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

  //command for establishing the setpoint in inches
  public Command setPositionCommand(double position){
    return runOnce(() -> m_setpoint = position / (Constants.Elevator.kElevatorPulleyDiameter * Math.PI));
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
