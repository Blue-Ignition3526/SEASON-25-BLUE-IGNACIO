package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbertakeConstants;

public class ClimbertakeRollers extends SubsystemBase {
  private final SparkFlex rollers;
  private final SparkFlexConfig config;

  public ClimbertakeRollers() {
    // Create roller motor
    rollers = new SparkFlex(ClimbertakeConstants.Rollers.kRollersMotorID, MotorType.kBrushless);

    // Configure roller motor
    config = new SparkFlexConfig();
    config
      .openLoopRampRate(ClimbertakeConstants.Rollers.kRollersMotorRampRate)
      .closedLoopRampRate(ClimbertakeConstants.Rollers.kRollersMotorRampRate)
      .smartCurrentLimit(ClimbertakeConstants.Rollers.kRollersMotorCurrentLimit)
      .voltageCompensation(12);

    // Apply config
    rollers.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
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

  public Command setInCommand() {
    return runOnce(this::setIn);
  }

  public Command setOutCommand() {
    return runOnce(this::setOut);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
