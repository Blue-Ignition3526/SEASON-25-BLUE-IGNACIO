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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveDrive;
import static frc.robot.Constants.SwerveDriveConstants.*;

public class DriveSwerve extends Command {

  //* The swerve drive subsystem
  private final SwerveDrive swerveDrive;

  //* The suppliers for the joystick values
  private final Supplier<Double> xSpeed;
  private final Supplier<Double> ySpeed;
  private final Supplier<Double> rotSpeed;
  private final Supplier<Boolean> fieldRelative;
  
  public double modifyAxis(double input, double scaleFactor, SlewRateLimiter slew) {
    // Get sign
    double sign = Math.signum(input);

    // Use absolute value for calculations (needed so slew rate limiter acts correctly against negative acceleration)
    input = Math.abs(input);
    
    // Apply deadband
    input = input < Constants.SwerveDriveConstants.kJoystickDeadband ? 0 : input;

    // Scale input
    input *= scaleFactor;

    // Apply rate limit
    slew.reset(input); // TODO: Check this
    input = slew.calculate(input);

    // Reapply the sign
    input *= sign;

    // Return the result
    return input;
  }

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
    double x = modifyAxis(xSpeed.get(), Constants.SwerveDriveConstants.PhysicalModel.kMaxSpeed.in(MetersPerSecond), xLimiter);
    double y = modifyAxis(ySpeed.get(), Constants.SwerveDriveConstants.PhysicalModel.kMaxSpeed.in(MetersPerSecond), yLimiter);
    double rot = modifyAxis(rotSpeed.get(), Constants.SwerveDriveConstants.PhysicalModel.kMaxAngularSpeed.in(RadiansPerSecond), rotLimiter);

    SmartDashboard.putNumber(getName() + "/X", x);
    SmartDashboard.putNumber(getName() + "/Y", y);
    SmartDashboard.putNumber(getName() + "/Rot", rot);
    
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
