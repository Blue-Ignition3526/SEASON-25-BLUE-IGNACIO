package frc.robot.speedAlterators;

import java.util.function.Supplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import lib.BlueShift.control.SpeedAlterator;
import frc.robot.Constants;

public class LookTowards extends SpeedAlterator {
    private final Supplier<Double> angleSupplier;

    private final Supplier<Double> targetAngle;

    public LookTowards(Supplier<Double> angleSupplier, Supplier<Double> targetAngle) {
        this.angleSupplier = angleSupplier;
        this.targetAngle = targetAngle;
        SmartDashboard.putBoolean("Alterators/Look", false);
    }
    
    @Override
    public void onEnable() {
        SmartDashboard.putBoolean("Alterators/Look", true);
    }

    @Override
    public void onDisable() {
        SmartDashboard.putBoolean("Alterators/Look", false);
    }

    public ChassisSpeeds alterSpeed(ChassisSpeeds speeds, boolean robotRelative) {
        double speed = -Constants.SwerveDrive.turningPID.calculate(angleSupplier.get() % 1, targetAngle.get() % 1);
        SmartDashboard.putNumber("Alterators/DesiredAngle", targetAngle.get() % 1);
        SmartDashboard.putNumber("Alterators/CurrentAngle", angleSupplier.get() % 1);
        SmartDashboard.putNumber("Alterators/Speed", speed);

        return new ChassisSpeeds(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speed);
    }
}