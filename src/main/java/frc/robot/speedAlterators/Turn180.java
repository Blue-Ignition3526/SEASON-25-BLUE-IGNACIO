package frc.robot.speedAlterators;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import lib.BlueShift.control.SpeedAlterator;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveDrive;

public class Turn180 extends SpeedAlterator {
    private final SwerveDrive swerveDrive;

    private double targetAngle;

    public Turn180(SwerveDrive swerveDrive) {
        this.swerveDrive = swerveDrive;
        SmartDashboard.putBoolean("Alterators/turning180", false);
        Constants.SwerveDriveConstants.PoseControllers.rotationPID.enableContinuousInput(-0.5, 0.5);
    }
    
    @Override
    public void onEnable() {
        targetAngle = swerveDrive.odometry.getPoseMeters().getRotation().rotateBy(Rotation2d.k180deg).getRotations();
        SmartDashboard.putBoolean("Alterators/turning180", true);
    }

    @Override
    public void onDisable() {
        SmartDashboard.putBoolean("Alterators/turning180", false);
    }

    public ChassisSpeeds alterSpeed(ChassisSpeeds speeds, boolean robotRelative) {
        double speed = Constants.SwerveDriveConstants.PoseControllers.rotationPID.calculate((swerveDrive.odometry.getPoseMeters().getRotation().getRotations()), targetAngle);
        SmartDashboard.putNumber("Alterators/DesiredAngle", targetAngle);
        SmartDashboard.putNumber("Alterators/CurrentAngle", (swerveDrive.odometry.getPoseMeters().getRotation().getRotations()));
        SmartDashboard.putNumber("Alterators/Speed", speed);

        speeds.omegaRadiansPerSecond = speed;

        return speeds;
    }
}