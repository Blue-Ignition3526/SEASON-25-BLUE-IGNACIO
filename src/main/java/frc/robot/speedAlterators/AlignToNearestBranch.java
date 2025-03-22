package frc.robot.speedAlterators;

import java.util.ArrayList;
import java.util.function.Supplier;

//import org.dyn4j.geometry.Transform;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.SwerveDriveConstants;
import lib.BlueShift.control.SpeedAlterator;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.FieldConstants;

public class AlignToNearestBranch extends SpeedAlterator {
    private final Supplier<Pose2d> poseSupplier;
    private final Supplier<Boolean> isRightSupplier;
    private final Supplier<Double> xOffsetSupplier;
    private final Supplier<Double> yOffsetSupplier;
    private Alliance alliance = Alliance.Blue;

    public AlignToNearestBranch(Supplier<Pose2d> poseSupplier, Supplier<Boolean> isRightSupplier, Supplier<Double> xOffsetSupplier, Supplier<Double> yOffsetSupplier) {
        this.poseSupplier = poseSupplier;
        this.isRightSupplier = isRightSupplier;
        this.xOffsetSupplier = xOffsetSupplier;
        this.yOffsetSupplier = yOffsetSupplier;
    }

    @Override
    public void onEnable() {
        Pose2d pose = poseSupplier.get();
        SwerveDriveConstants.PoseControllers.translationXPID.reset(pose.getX());
        SwerveDriveConstants.PoseControllers.translationYPID.reset(pose.getY());
        SwerveDriveConstants.PoseControllers.rotationPID.reset(pose.getRotation().getRotations());

        this.alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
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
        pose.transformBy(new Transform2d(AllianceFlipUtil.apply(new Translation2d(
            SwerveDriveConstants.PoseControllers.kOffsetBoxWidth.times(xOffsetSupplier.get()),
            SwerveDriveConstants.PoseControllers.kOffsetBoxWidth.times(yOffsetSupplier.get())
        ), false), Rotation2d.kZero));

        // Get nearest reef face center (thresholding)
        Pose2d nearestFaceCenter = pose.nearest(getAppropriateReefFaceCenters(pose));

        // Log
        Logger.recordOutput("Automation/NearestReefFaceCenter", nearestFaceCenter);

        return speeds;
    }
}
