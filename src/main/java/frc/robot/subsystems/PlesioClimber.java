package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PlesioClimber extends SubsystemBase {
  private final Servo servo;
  
  /** Our climber modeled after a plesiosaurus */
  public PlesioClimber() {
    this.servo = new Servo(1);

    servo.setBoundsMicroseconds(1000, 10, 1500, 10, 2000);    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
