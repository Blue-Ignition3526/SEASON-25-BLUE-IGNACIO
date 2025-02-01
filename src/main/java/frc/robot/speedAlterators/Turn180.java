package frc.robot.speedAlterators;

import java.util.function.Supplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import lib.BlueShift.control.SpeedAlterator;
import frc.robot.Constants;

public class Turn180 extends SpeedAlterator {
    private final Supplier<Double> angleSupplier;

    private double targetAngle;

    public Turn180(Supplier<Double> angleSupplier) {
        this.angleSupplier = angleSupplier;
        SmartDashboard.putBoolean("Alterators/turning180", false);
        Constants.SwerveDrive.PoseControllers.turningPID.enableContinuousInput(-0.5, 0.5);
    }
    
    @Override
    public void onEnable() {
        targetAngle = (angleSupplier.get() + 0.5) % 1;
        SmartDashboard.putBoolean("Alterators/turning180", true);
    }

    @Override
    public void onDisable() {
        SmartDashboard.putBoolean("Alterators/turning180", false);
    }

    public ChassisSpeeds alterSpeed(ChassisSpeeds speeds, boolean robotRelative) {
        double speed = -Constants.SwerveDrive.PoseControllers.turningPID.calculate((angleSupplier.get()) % 1, targetAngle);
        SmartDashboard.putNumber("Alterators/DesiredAngle", targetAngle);
        SmartDashboard.putNumber("Alterators/CurrentAngle", (angleSupplier.get()) % 1);
        SmartDashboard.putNumber("Alterators/Speed", speed);

        return new ChassisSpeeds(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speed);
    }
}