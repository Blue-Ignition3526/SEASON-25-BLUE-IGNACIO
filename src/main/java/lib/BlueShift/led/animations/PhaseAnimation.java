package lib.BlueShift.led.animations;

import edu.wpi.first.wpilibj.Timer;
import lib.BlueShift.led.framework.HyperAddressableLEDBuffer;
import lib.BlueShift.led.framework.HyperLEDAnimation;

public class PhaseAnimation extends HyperLEDAnimation {
    int r;
    int g;
    int b;
    double duration;

    public PhaseAnimation(int r, int g, int b, double duration) {
        this.r = r;
        this.g = g;
        this.b = b;
        this.duration = duration;
    }

    public void provider(HyperAddressableLEDBuffer data) {
        double timeFraction = 1 - (Timer.getFPGATimestamp() % duration);
        int center = (int)Math.floor(data.getLength() / 2);
        for (int i = 0; i < data.getLength(); i++) {
            double distance = Math.abs(center - i);
            double brightness = 1 - (distance / center);
            data.setRGB(
                i,
                (int)Math.floor(r * brightness * timeFraction),
                (int)Math.floor(g * brightness * timeFraction),
                (int)Math.floor(b * brightness * timeFraction)
            );
        }
    }
}
