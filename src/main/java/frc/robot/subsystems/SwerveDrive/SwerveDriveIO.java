package frc.robot.subsystems.SwerveDrive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import lib.BlueShift.control.SpeedAlterator;

public interface SwerveDriveIO {
    public Rotation2d getHeading();
    public void zeroHeading();

    public ChassisSpeeds getRobotRelativeChassisSpeeds();
    
    public SwerveModuleState[] getModuleTargetStates();
    public SwerveModuleState[] getModuleRealStates();
    public SwerveModulePosition[] getModulePositions();

    public void setModuleStates(SwerveModuleState[] states, boolean force);
    public default void setModuleStates(SwerveModuleState[] states) {
        setModuleStates(states, false);
    };

    public void enableSpeedAlterator(SpeedAlterator alterator);
    public void disableSpeedAlterator();

    public void driveRobotRelative(ChassisSpeeds speeds);
    public void driveRobotRelative(double xSpeed, double ySpeed, double rotSpeed);

    public void driveFieldRelative(ChassisSpeeds speeds);
    public void driveFieldRelative(double xSpeed, double ySpeed, double rotSpeed);

    public void stop();

    public default void xFormation() {
        setModuleStates(new SwerveModuleState[]{
            new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
            new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
            new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
            new SwerveModuleState(0, Rotation2d.fromDegrees(-45))
        }, true);
    }

    public void resetTurningEncoders();
    public void resetDriveEncoders();

    public default void resetEncoders() {
        resetTurningEncoders();
        resetDriveEncoders();
    }

    public default void periodic() {};
}
