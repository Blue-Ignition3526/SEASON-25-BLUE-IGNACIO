package lib.team3526.math;

import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.Measure;
import edu.wpi.first.wpilibj.Timer;

public class RotationalInertiaAccumulator {
    /**
     * RADIANS
     */
    private double previousYaw = 0;
    private double previousTime = 0;
    private double totalRotationalInertia = 0;

    /**
     * KILOGRAMS
     */
    private double mass;

    public RotationalInertiaAccumulator(double massKg, Measure<AngleUnit> yaw) {
        this.mass = massKg;
        this.previousYaw = yaw.in(Radians);
    }

    public RotationalInertiaAccumulator(double massKg) {
        this(massKg, Radians.zero());
    }

    public void update(double currentYaw) {
        reset();

        double currentTime = Timer.getFPGATimestamp();

        if (previousTime == 0) {
            // If this is the first update, simply update the previous values
            previousYaw = currentYaw;
            previousTime = currentTime;
            return;
        }

        // Calculate change in yaw and change in time
        double deltaYaw = currentYaw - previousYaw;
        double deltaTime = currentTime - previousTime;

        // Calculate angular velocity (in radians per second)
        double angularVelocity = deltaYaw / deltaTime;

        // Calculate rotational inertia using formula: torque = moment of inertia * angular acceleration
        double rotationalInertia = mass * angularVelocity;

        // Accumulate rotational inertia
        totalRotationalInertia += rotationalInertia;

        // Update previous values for next iteration
        previousYaw = currentYaw;
        previousTime = currentTime;
    }

    public double getTotalRotationalInertia() {
        return totalRotationalInertia;
    }

    public void reset() {
        totalRotationalInertia = 0;
    }
}
