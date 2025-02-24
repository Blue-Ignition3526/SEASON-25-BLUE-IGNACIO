package frc.robot.speedAlterators;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import lib.BlueShift.control.SpeedAlterator;
import frc.robot.Constants;

public class Turn180 extends SpeedAlterator {
    private final Supplier<Pose2d> poseSupplier;
    private double targetAngle;

    public Turn180(Supplier<Pose2d> poseSupplier) {
        this.poseSupplier = poseSupplier;
        SmartDashboard.putBoolean("Alterators/turning180", false);
        Constants.SwerveDriveConstants.PoseControllers.rotationPID.enableContinuousInput(-0.5, 0.5);
    }
    
    @Override
    public void onEnable() {
        targetAngle = poseSupplier.get().getRotation().rotateBy(Rotation2d.k180deg).getRotations();
        SmartDashboard.putBoolean("Alterators/turning180", true);
    }

    @Override
    public void onDisable() {
        SmartDashboard.putBoolean("Alterators/turning180", false);
    }

    public ChassisSpeeds alterSpeed(ChassisSpeeds speeds, boolean robotRelative) {
        double speed = Constants.SwerveDriveConstants.PoseControllers.rotationPID.calculate((poseSupplier.get().getRotation().getRotations()), targetAngle);
        SmartDashboard.putNumber("Alterators/DesiredAngle", targetAngle);
        SmartDashboard.putNumber("Alterators/CurrentAngle", (poseSupplier.get().getRotation().getRotations()));
        SmartDashboard.putNumber("Alterators/Speed", speed);

        speeds.omegaRadiansPerSecond = speed;

        return speeds;
    }
}