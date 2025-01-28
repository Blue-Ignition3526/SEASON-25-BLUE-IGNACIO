package frc.robot.speedAlterators;

import java.util.function.Supplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import lib.BlueShift.control.SpeedAlterator;
import lib.BlueShift.math.BlueMathUtils;

public class LookController extends SpeedAlterator {
    private final Supplier<Double> x;
    private final Supplier<Double> y;
    private final Supplier<Double> angleSupplier;
    private final double deadzone;

    public LookController(Supplier<Double> angleSupplier, Supplier<Double> controllerX, Supplier<Double> controllerY, double deadzone) {
        this.angleSupplier = angleSupplier;
        this.x = controllerX;
        this.y = controllerY;
        this.deadzone = deadzone;
    }

    @Override
    public void onEnable() {

    }

    @Override
    public void onDisable() {
        
    }

    public ChassisSpeeds alterSpeed(ChassisSpeeds speeds, boolean robotRelative) {
        if(Math.abs(x.get()) < deadzone && Math.abs(y.get()) < deadzone) return speeds;

        double angle = -(Math.atan2(x.get(), y.get()) / (Math.PI*2)) % 1;

        double speed = -Constants.SwerveDrive.turningPID.calculate(angleSupplier.get() % 1, angle);
        SmartDashboard.putNumber("Alterators/LookJoystick/DesiredAngle", angle);
        SmartDashboard.putNumber("Alterators/LookJoystick/CurrentAngle", angleSupplier.get() % 1);
        SmartDashboard.putNumber("Alterators/LookJoystick/Speed", speed);

        return new ChassisSpeeds(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speed);
    }
}