package lib.team3526.utils;

import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

public class BluePWMEncoder extends DutyCycleEncoder {
    private double offset = 0;

    /**
     * Blueshift-extended PWM encoder
     * <p>
     * Added features
     * <p>
     * - Offsets
     * <p>
     * - Velocity calculation
     * <p>
     * - Units
     * @param channel
     */
    public BluePWMEncoder(int channel) {
        super(channel);
    }

    /**
     * Adds an offset to the encoder
     * @param offset
     */
    public void setOffset(double offset) {
        this.offset = MathUtil.clamp(offset, 0, 1);
    }
    
    /**
     * Returns the angle without an offset
     * @return
     */
    public Angle getAngleNoOffset() {
        return Rotations.of(get());
    }

    /**
     * Returns the angle
     * @return
     */
    public Angle getAngle() {
        return Rotations.of(MathUtil.inputModulus(get() + offset, 0, 1));
    }
}
