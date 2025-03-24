package frc.robot.speedAlterators;

import java.util.ArrayList;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.SwerveDriveConstants;
import lib.BlueShift.control.SpeedAlterator;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.FieldConstants;

public class AlignToNearestBranch extends SpeedAlterator {
    private final Supplier<Pose2d> poseSupplier;
    private final Supplier<Boolean> isRightSupplier;
    private final Supplier<Double> xOffsetSupplier;
    private final Supplier<Double> yOffsetSupplier;

    public AlignToNearestBranch(Supplier<Pose2d> poseSupplier, Supplier<Boolean> isRightSupplier, Supplier<Double> xOffsetSupplier, Supplier<Double> yOffsetSupplier) {
        this.poseSupplier = poseSupplier;
        this.isRightSupplier = isRightSupplier;
        this.xOffsetSupplier = xOffsetSupplier;
        this.yOffsetSupplier = yOffsetSupplier;
    }

    @Override
    public void onEnable() {
        Pose2d pose = poseSupplier.get();
        // SwerveDriveConstants.PoseControllers.translationXPID.reset(pose.getX());
        // SwerveDriveConstants.PoseControllers.translationYPID.reset(pose.getY());
        // SwerveDriveConstants.PoseControllers.rotationPID.reset(pose.getRotation().getRotations());
    }

    ArrayList<Pose2d> getAppropriateReefFaceCenters(Pose2d pose) {
        if (pose.getX() < FieldConstants.Reef.driverFacingLine) return FieldConstants.Reef.driverFacingThresholdingCenters;
        else return FieldConstants.Reef.centerFacingThresholdingCenters;
    }

    @Override
    public ChassisSpeeds alterSpeed(ChassisSpeeds speeds, boolean robotRelative) {
        // Get robot pose
        Pose2d pose = poseSupplier.get();

        // Add offset to pose
        Pose2d thresholdingPose = new Pose2d(pose.getTranslation(), pose.getRotation()).transformBy(new Transform2d(AllianceFlipUtil.apply(new Translation2d(
            SwerveDriveConstants.PoseControllers.kOffsetBoxWidth.times(xOffsetSupplier.get()),
            SwerveDriveConstants.PoseControllers.kOffsetBoxWidth.times(yOffsetSupplier.get())
        ), false), Rotation2d.kZero));

        // Get nearest reef face center (thresholding)
        // TODO: Check if this works with red
        Pose2d nearestFaceCenter = pose.nearest(getAppropriateReefFaceCenters(pose));

        // Apply offset to branch (either left or right depending on the value returned by the isRight supplier)
        // TODO: Currently it will go to the thresholding pose, offset it to be up against the reef face
        Pose2d poseAlignedToBranch = AllianceFlipUtil.apply(new Pose2d(
            new Translation2d(
                // TODO: wtf 
                nearestFaceCenter
                    .transformBy(new Transform2d(0.0, -(FieldConstants.Reef.branchAdjustY + 0.10) * (isRightSupplier.get() ? 1 : -1), Rotation2d.kZero))
                    .getX(),
                nearestFaceCenter
                    .transformBy(new Transform2d(0.0, -(FieldConstants.Reef.branchAdjustY + 0.10) * (isRightSupplier.get() ? 1 : -1), Rotation2d.kZero))
                    .getY()
            ),
            nearestFaceCenter.getRotation()
        ), false);

        // Log
        Logger.recordOutput("Automation/ThresholdingPose", thresholdingPose);
        Logger.recordOutput("Automation/NearestReefFaceCenter", nearestFaceCenter);
        Logger.recordOutput("Automation/AlignedToBranch", poseAlignedToBranch);

        double vX = SwerveDriveConstants.PoseControllers.translationXPID.calculate(pose.getX(), poseAlignedToBranch.getX());
        SmartDashboard.putNumber("PLSPLS/Vx", vX);
        double vY = SwerveDriveConstants.PoseControllers.translationYPID.calculate(pose.getY(), poseAlignedToBranch.getY());
        SmartDashboard.putNumber("PLSPLS/Vy", vY);
        double rot = SwerveDriveConstants.PoseControllers.rotationPID.calculate(pose.getRotation().getRotations(), poseAlignedToBranch.getRotation().getRotations());

        ChassisSpeeds newSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(vX, vY, rot, pose.getRotation());
        
        SmartDashboard.putString("Automation/TargetSpeeds", newSpeeds.toString());

        return newSpeeds;
    }
}
