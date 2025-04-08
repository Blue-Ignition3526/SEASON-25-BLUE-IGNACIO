package frc.robot.subsystems.SwerveDrive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import lib.BlueShift.control.SpeedAlterator;

public class SwerveDrive extends SubsystemBase implements SwerveDriveIO {
  SwerveDriveIO io;

  public SwerveDrive(SwerveDriveIO io) {
    this.io = io;

    SmartDashboard.putData("Swerve/ZeroHeading", zeroHeadingCommand());
  }

  public Rotation2d getHeading() {
    return io.getHeading();
  }

  public void zeroHeading() {
    io.zeroHeading();
  }

  public ChassisSpeeds getRobotRelativeChassisSpeeds() {
    return io.getRobotRelativeChassisSpeeds();
  }

  public SwerveModuleState[] getModuleTargetStates() {
    return io.getModuleTargetStates();
  }

  public SwerveModuleState[] getModuleRealStates() {
    return io.getModuleRealStates();
  }

  public SwerveModulePosition[] getModulePositions() {
    return io.getModulePositions();
  }

  public void setModuleStates(SwerveModuleState[] states) {
    io.setModuleStates(states);
  }

  public void drive(ChassisSpeeds speeds) {
    io.drive(speeds);
  }

  public void enableSpeedAlterator(SpeedAlterator alterator) {
    io.enableSpeedAlterator(alterator);
  }

  public void disableSpeedAlterator() {
    io.disableSpeedAlterator();
  }

  public void driveFieldRelative(double vx, double vy, double omega) {
    io.driveFieldRelative(vx, vy, omega);
  }

  public void driveFieldRelative(ChassisSpeeds speeds) {
    io.driveFieldRelative(speeds);
  }

  public void driveRobotRelative(double vx, double vy, double omega) {
    io.driveRobotRelative(vx, vy, omega);
  }

  public void driveRobotRelative(ChassisSpeeds speeds) {
    io.driveRobotRelative(speeds);
  }

  public void stop() {
    io.stop();
  }

  public void xFormation() {
    io.xFormation();
  }

  public void resetTurningEncoders() {
    io.resetTurningEncoders();
  }

  public void resetDriveEncoders() {
    io.resetDriveEncoders();
  }

  public void resetEncoders() {
    io.resetEncoders();
  }

  // ! COMMANDS
  /**
   * Resets the gyroscope
   * @return
   */
  public Command zeroHeadingCommand() {
    return runOnce(this::zeroHeading).ignoringDisable(true);
  }

  /**
   * Enable a speed alterator with a command
   * @param alterator
   * @return
   */
  public Command enableSpeedAlteratorCommand(SpeedAlterator alterator) {
      return runOnce(() -> this.enableSpeedAlterator(alterator));
  }

  /**
     * Disable the speed alterator with a command
     * @return
     */
    public Command disableSpeedAlteratorCommand() {
      return runOnce(() -> this.disableSpeedAlterator());
  }
  
  @Override
  public void periodic() {
    io.periodic();
  }
}
