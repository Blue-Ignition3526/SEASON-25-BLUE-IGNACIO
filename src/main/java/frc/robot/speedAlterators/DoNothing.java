package frc.robot.speedAlterators;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import lib.BlueShift.control.SpeedAlterator;
import frc.robot.subsystems.SwerveDrive;

public class DoNothing extends SpeedAlterator {
    private final SwerveDrive swerve;

    public DoNothing(SwerveDrive swerve) {
        this.swerve = swerve;
    }

    @Override
    public void onEnable() {
        Logger.recordOutput("PrePose", swerve.getPose());
        SmartDashboard.putBoolean("Alterator/not", true);
    }
    @Override
    public void onDisable() {
        SmartDashboard.putBoolean("Alterator/not", false);
        i=true;
    }

    static boolean i = true;
    public ChassisSpeeds alterSpeed(ChassisSpeeds speeds, boolean robotRelative) {
        if(i) {
            Logger.recordOutput("PrePose", swerve.getPose());
            i=false;
        }
        return speeds;
    }
}
