package frc.robot.subsystems.Gyro;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import org.ironmaple.simulation.drivesims.GyroSimulation;
import edu.wpi.first.math.geometry.Rotation2d;

public class GyroIOSim implements GyroIO {
    private final GyroSimulation simulation;

    public GyroIOSim(GyroSimulation simulation) {
        this.simulation = simulation;
    }

    public double getPitch() {
        return 0;
    }

    public double getYaw() {
        return simulation.getGyroReading().getRotations();
    }

    public double getRoll() {
        return 0;
    }

    public double getPitchVelocity() {
        return 0;
    }

    public double getYawVelocity() {
        return simulation.getMeasuredAngularVelocity().in(RotationsPerSecond);
    }

    public double getRollVelocity() {
        return 0;
    }

    public double getAccelerationX() {
        return 0;
    }

    public double getAccelerationY() {
        return 0;
    }

    public double getAccelerationZ() {
        return 0;
    }

    public double getVelocityX() {
        return 0;
    }

    public double getVelocityY() {
        return 0;
    }

    public double getVelocityZ() {
        return 0;
    }

    public Rotation2d getHeading() {
        return simulation.getGyroReading();
    }

    public void reset() {
        simulation.setRotation(Rotation2d.fromDegrees(0));
    }
}
