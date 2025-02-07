// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Second;

import frc.robot.Constants;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveDrive;

public class DriveSwerve extends Command {

  //* The swerve drive subsystem
  private final SwerveDrive swerveDrive;

  //* Slew rate limiters
  SlewRateLimiter xLimiter = new SlewRateLimiter(Constants.SwerveDriveConstants.PhysicalModel.kMaxAcceleration.in(MetersPerSecondPerSecond));
  SlewRateLimiter yLimiter = new SlewRateLimiter(Constants.SwerveDriveConstants.PhysicalModel.kMaxAcceleration.in(MetersPerSecondPerSecond));
  SlewRateLimiter rotLimiter = new SlewRateLimiter(Constants.SwerveDriveConstants.PhysicalModel.kMaxAngularAcceleration.in(RadiansPerSecond.per(Second)));

  //* The suppliers for the joystick values
  private final Supplier<Double> xSpeed;
  private final Supplier<Double> ySpeed;
  private final Supplier<Double> rotSpeed;
  private final Supplier<Boolean> fieldRelative;

  public DriveSwerve(SwerveDrive swerveDrive, Supplier<Double> x, Supplier<Double> y, Supplier<Double> rot, Supplier<Boolean> fieldRelative) {
    this.swerveDrive = swerveDrive;
    this.xSpeed = x;
    this.ySpeed = y;
    this.rotSpeed = rot;
    this.fieldRelative = fieldRelative;
    addRequirements(swerveDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Get the joystick values
    double x = xSpeed.get();
    double y = ySpeed.get();
    double rot = rotSpeed.get();

    // Apply deadzone to the joystick values
    x = Math.abs(x) < Constants.SwerveDriveConstants.kJoystickDeadband ? 0 : x;
    y = Math.abs(y) < Constants.SwerveDriveConstants.kJoystickDeadband ? 0 : y;
    rot = Math.abs(rot) < Constants.SwerveDriveConstants.kJoystickDeadband ? 0 : rot;
    
    // Apply the slew rate limiters
    x = xLimiter.calculate(x);
    y = yLimiter.calculate(y);
    rot = rotLimiter.calculate(rot);

    // Scale the joystick values to the max speed
    x *= Constants.SwerveDriveConstants.PhysicalModel.kMaxSpeed.in(MetersPerSecond);
    y *= Constants.SwerveDriveConstants.PhysicalModel.kMaxSpeed.in(MetersPerSecond);
    rot *= Constants.SwerveDriveConstants.PhysicalModel.kMaxAngularSpeed.in(RadiansPerSecond);
    
    // Drive the swerve drive
    if (this.fieldRelative.get()) {
      swerveDrive.driveFieldRelative(x, y, rot);
    } else {
      swerveDrive.driveRobotRelative(x, y, rot);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerveDrive.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
