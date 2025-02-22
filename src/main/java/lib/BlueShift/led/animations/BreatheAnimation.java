package lib.BlueShift.led.animations;

import edu.wpi.first.wpilibj.Timer;
import lib.BlueShift.led.framework.HyperAddressableLEDBuffer;
import lib.BlueShift.led.framework.HyperLEDAnimation;

public class BreatheAnimation extends HyperLEDAnimation {
    int r;
    int g;
    int b;
    double duration;

    public BreatheAnimation(int r, int g, int b, double duration) {
        this.r = r;
        this.g = g;
        this.b = b;
        this.duration = duration;
    }

    public void provider(HyperAddressableLEDBuffer data) {
        double timeFraction = 1 - (Timer.getFPGATimestamp() % duration);
        for (int i = 0; i < data.getLength(); i++) data.setRGB(
            i,
            (int)Math.floor(r * timeFraction),
            (int)Math.floor(g * timeFraction),
            (int)Math.floor(b * timeFraction)
        );
    }
}
