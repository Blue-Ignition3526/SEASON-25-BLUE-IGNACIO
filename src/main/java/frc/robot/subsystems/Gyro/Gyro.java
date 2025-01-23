// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Gyro;

// import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Gyro extends SubsystemBase implements GyroIO {
  GyroIO io;
  // GyroIOInputsAutoLogged inputs = new GyroIOInputsAutoLogged();

  /**
   * Creates a new Gyro. 
   **/
  public Gyro(GyroIO io) {
    this.io = io;
  }

  /**
   * Gets the pitch of the gyro.
   * @return
   */
  public double getPitch() {
    return io.getPitch();
  }

  /**
   * Gets the yaw of the gyro.
   * @return
   */
  public double getYaw() {
    return io.getYaw();
  }

  /**
   * Gets the roll of the gyro.
   * @return
   */
  public double getRoll() {
    return io.getRoll();
  }

  /**
   * Gets the pitch velocity of the gyro.
   * @return
   */ 
  public double getPitchVelocity() {
    return io.getPitchVelocity();
  }

  /**
   * Gets the yaw velocity of the gyro.
   * @return
   */
  public double getYawVelocity() {
    return io.getYawVelocity();
  }

  /**
   * Gets the roll velocity of the gyro.
   * @return
   */
  public double getRollVelocity() {
    return io.getRollVelocity();
  }

  /**
   * Gets the acceleration of the gyro in the X direction.
   * @return
   */
  public double getAccelerationX() {
    return io.getAccelerationX();
  }

  /**
   * Gets the acceleration of the gyro in the Y direction.
   * @return
   */
  public double getAccelerationY() {
    return io.getAccelerationY();
  }

  /**
   * Gets the acceleration of the gyro in the Z direction.
   * @return
   */
  public double getAccelerationZ() {
    return io.getAccelerationZ();
  }

  public Rotation2d getHeading() {
    return io.getHeading();
  }

  /**
   * Resets the gyro.
   */
  public void reset() {
    io.reset();
  }

  /**
   * Gets the velocity of the gyro in the X direction.
   */
  public double getVelocityX() {
    return io.getVelocityX();
  }

  /**
   * Gets the velocity of the gyro in the Y direction.
   */
  public double getVelocityY() {
    return io.getVelocityY();
  }

  /**
   * Gets the velocity of the gyro in the Z direction.
   */
  public double getVelocityZ() {
    return io.getVelocityZ();
  }

  @Override
  public void periodic() {
    io.periodic();
    // io.updateInputs(inputs);

    // Logger.recordOutput("Gyro/Pitch", getPitch());
    // Logger.recordOutput("Gyro/Yaw", getYaw());
    // Logger.recordOutput("Gyro/Roll", getRoll());
  }
}
