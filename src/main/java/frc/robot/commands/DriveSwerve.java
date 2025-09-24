package frc.robot.commands;

import java.util.function.Supplier;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import frc.robot.Constants;
import frc.robot.Constants.SwerveDriveConstants;
import frc.robot.subsystems.SwerveDrive.SwerveDrive;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;

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
    // ! Deadband applied to joystick directly
    // input = input > Constants.SwerveDriveConstants.kJoystickDeadband ? input : 0;

    // Scale input
    input *= scaleFactor;

    // Square input
    //! CHECK THIS
    //input *= input;

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
    double x = xSpeed.get();
    double y = ySpeed.get();
    double rot = rotSpeed.get();

    // Apply deadzone to the joystick values
    if(Math.hypot(x, y) > Constants.SwerveDriveConstants.kJoystickDeadband) {
      //x = (x + ((x > 0 ? -1 : 1) * Constants.SwerveDriveConstants.kJoystickDeadband)) * 1 / ( 1 - Constants.SwerveDriveConstants.kJoystickDeadband);
      //y = (y + ((y > 0 ? -1 : 1) * Constants.SwerveDriveConstants.kJoystickDeadband)) * 1 / ( 1 - Constants.SwerveDriveConstants.kJoystickDeadband);
    } else {
      x = 0;
      y = 0;
    }
    
    rot = Math.abs(rot) < Constants.SwerveDriveConstants.kJoystickDeadband ? 0 : rot;

    // Modify the axis
    x = modifyAxis(x, Constants.SwerveDriveConstants.PhysicalModel.kMaxSpeed.in(MetersPerSecond), SwerveDriveConstants.PhysicalModel.xLimiter);
    y = modifyAxis(y, Constants.SwerveDriveConstants.PhysicalModel.kMaxSpeed.in(MetersPerSecond), SwerveDriveConstants.PhysicalModel.yLimiter);
    rot = modifyAxis(rot, Constants.SwerveDriveConstants.PhysicalModel.kMaxAngularSpeed.in(RadiansPerSecond), SwerveDriveConstants.PhysicalModel.rotLimiter);
    
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
