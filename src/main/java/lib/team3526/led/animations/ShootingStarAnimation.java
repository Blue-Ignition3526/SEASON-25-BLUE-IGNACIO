package lib.team3526.led.animations;

import edu.wpi.first.wpilibj.Timer;
import lib.team3526.led.framework.HyperAddressableLEDBuffer;
import lib.team3526.led.framework.HyperLEDAnimation;

public class ShootingStarAnimation extends HyperLEDAnimation {
    int h;
    int s;
    int v;
    int trailLength;
    double duration;

    public ShootingStarAnimation(int h, int s, int v, int trailLength, double duration) {
        this.h = h;
        this.s = s;
        this.v = v;
        this.trailLength = trailLength;
        this.duration = duration;
    }

    public void provider(HyperAddressableLEDBuffer data) {
        double timeFraction = (Timer.getFPGATimestamp() % duration) / duration;
        int trailStart = (int)(timeFraction * data.getLength());

        // Write the trail
        // If the index is equal to the trail start, it is fully bright
        // If the index is less than the trail start, it is dimmer depending on how far from the trail start it is
        for (int i = 0; i < data.getLength(); i++) {
            int distance = Math.abs(i - trailStart);
            int brightness = 255 - (int)(255 * (distance / (double)trailLength));
            data.setHSV(i, h, s, brightness);
        }
    }
}
