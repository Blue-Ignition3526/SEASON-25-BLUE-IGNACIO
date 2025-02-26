package frc.robot.speedAlterators;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import lib.BlueShift.control.SpeedAlterator;

public class BackUp extends SpeedAlterator {
    private final double speed;
    private final Supplier<Rotation2d> headingSupplier;

    public BackUp(double speed, Supplier<Rotation2d> headingSupplier) {
        this.speed = speed;
        this.headingSupplier = headingSupplier;
    }

    public ChassisSpeeds alterSpeed(ChassisSpeeds speeds, boolean robotRelative) {
        return ChassisSpeeds.fromRobotRelativeSpeeds(0, speed, 0, headingSupplier.get());
    }
}
