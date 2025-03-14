package frc.robot.speedAlterators;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import lib.BlueShift.control.SpeedAlterator;

public class GoToNearestBranch extends SpeedAlterator {
    private final Supplier<Pose2d> poseSupplier;

    public GoToNearestBranch() {
        this.poseSupplier = () -> new Pose2d();
    }
}
