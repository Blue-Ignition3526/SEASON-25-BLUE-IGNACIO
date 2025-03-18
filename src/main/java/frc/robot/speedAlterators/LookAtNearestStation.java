package frc.robot.speedAlterators;

import java.util.function.Supplier;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.FieldConstants;
import frc.robot.Constants.SwerveDriveConstants;
import lib.BlueShift.control.SpeedAlterator;

public class LookAtNearestStation extends SpeedAlterator {
    private final Supplier<Pose2d> poseSupplier;
    private Alliance alliance = Alliance.Blue;

    public LookAtNearestStation(Supplier<Pose2d> poseSupplier) {
        this.poseSupplier = poseSupplier;
    }

    @Override
    public void onEnable() {
        Pose2d pose = poseSupplier.get();
        SwerveDriveConstants.PoseControllers.translationXPID.reset(pose.getX());
        SwerveDriveConstants.PoseControllers.translationYPID.reset(pose.getY());
        SwerveDriveConstants.PoseControllers.rotationPID.reset(pose.getRotation().getRotations());

        this.alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
    }

    @Override
    public ChassisSpeeds alterSpeed(ChassisSpeeds speeds, boolean robotRelative) {
        // Get latest pose
        Pose2d pose = poseSupplier.get();
        
        // Check the side of the field the robot is on
        double fieldCenter = FieldConstants.kFieldCenter.getY();

        double targetRotation;
        if (pose.getY() > fieldCenter) targetRotation = -60;
        else targetRotation = 60;

        if (alliance == Alliance.Red) targetRotation += 180;

        // TODO: es muy tarde para mate, ya mañana.

        return speeds;
    }
}
