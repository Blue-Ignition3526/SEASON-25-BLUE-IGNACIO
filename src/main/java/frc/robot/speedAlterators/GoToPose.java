package frc.robot.speedAlterators;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import lib.BlueShift.control.SpeedAlterator;
import frc.robot.Constants;
import static frc.robot.Constants.SwerveDriveConstants.PoseControllers.epsilon;

public class GoToPose extends SpeedAlterator{
    private final Supplier<Pose2d> poseSupplier;
    private final Pose2d targetPose;

    public GoToPose(Supplier<Pose2d> poseSupllier, Pose2d targetPose) {
        this.poseSupplier = poseSupllier;
        this.targetPose = targetPose;
    }

    @Override
    public void onEnable() {
        Constants.SwerveDriveConstants.PoseControllers.translationPID.reset();
        Constants.SwerveDriveConstants.PoseControllers.rotationPID.reset();
    }

    @Override
    public void onDisable() {}

    public ChassisSpeeds alterSpeed(ChassisSpeeds speeds, boolean robotRelative) {
        Pose2d pose = poseSupplier.get();

        double xSpeed, ySpeed, rotSpeed;
        if(Math.abs(pose.getX() - targetPose.getX()) > epsilon) xSpeed = Constants.SwerveDriveConstants.PoseControllers.translationPID.calculate(pose.getX(), targetPose.getX());
        else xSpeed = speeds.vxMetersPerSecond;
        if(Math.abs(pose.getY() - targetPose.getY()) > epsilon)ySpeed = Constants.SwerveDriveConstants.PoseControllers.translationPID.calculate(pose.getY(), targetPose.getY());
        else ySpeed = speeds.vyMetersPerSecond;

        /*if(Math.abs(pose.getRotation().getRotations() - targetPose.getRotation().getRotations()) < rotEpsilon)*/ rotSpeed = Constants.SwerveDriveConstants.PoseControllers.rotationPID.calculate(pose.getRotation().getRotations(), targetPose.getRotation().getRotations());
        // else rotSpeed = speeds.omegaRadiansPerSecond;

        SmartDashboard.putNumber("Alterators/pose/rot", rotSpeed);
        SmartDashboard.putNumber("Alterators/pose/x", xSpeed);
        SmartDashboard.putNumber("Alterators/pose/y", ySpeed);
        Logger.recordOutput("Alterators/pose/desiredPose", targetPose);

        return ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rotSpeed, pose.getRotation());
    }
}
