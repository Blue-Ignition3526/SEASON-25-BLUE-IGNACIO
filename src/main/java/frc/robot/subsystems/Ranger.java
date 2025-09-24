package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Centimeters;

import com.ctre.phoenix6.hardware.CANrange;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Ranger extends SubsystemBase {
  private final CANrange leftRanger;
  private final CANrange rightRanger;

  public Ranger() {
    this.leftRanger = new CANrange(41, "*");
    this.rightRanger = new CANrange(42, "*");
  }

  public Distance getLeft() {
    return leftRanger.getDistance().refresh().getValue();
  }

  public Distance getRight() {
    return rightRanger.getDistance().refresh().getValue();
  }

  @Override
  public void periodic() {
    SmartDashboard.putString("Ranger/Left", String.format("%.4f", getLeft().in(Centimeters)));
    SmartDashboard.putString("Ranger/Right", String.format("%.4f", getRight().in(Centimeters)));
  }
}
