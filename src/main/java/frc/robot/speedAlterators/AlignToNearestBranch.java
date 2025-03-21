package frc.robot.speedAlterators;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.SwerveDriveConstants;
import lib.BlueShift.control.SpeedAlterator;

public class AlignToNearestBranch extends SpeedAlterator {
    private final Supplier<Pose2d> poseSupplier;
    private final Supplier<Boolean> isRightSupplier;
    private Alliance alliance = Alliance.Blue;

    public AlignToNearestBranch(Supplier<Pose2d> poseSupplier, Supplier<Boolean> isRightSupplier) {
        this.poseSupplier = poseSupplier;
        this.isRightSupplier = isRightSupplier;
    }

    @Override
    public void onEnable() {
        Pose2d pose = poseSupplier.get();
        SwerveDriveConstants.PoseControllers.translationXPID.reset(pose.getX());
        SwerveDriveConstants.PoseControllers.translationYPID.reset(pose.getY());
        SwerveDriveConstants.PoseControllers.rotationPID.reset(pose.getRotation().getRotations());

        this.alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
    }
}
