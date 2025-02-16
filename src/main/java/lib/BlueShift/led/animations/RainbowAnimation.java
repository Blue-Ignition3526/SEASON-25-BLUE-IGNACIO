package lib.BlueShift.led.animations;

import edu.wpi.first.wpilibj.Timer;
import lib.BlueShift.led.framework.HyperAddressableLEDBuffer;
import lib.BlueShift.led.framework.HyperLEDAnimation;

public class RainbowAnimation extends HyperLEDAnimation {
    double duration;

    public RainbowAnimation(double duration) {
        this.duration = duration;
    }

    public void provider(HyperAddressableLEDBuffer data) {
        double timeFraction = (Timer.getFPGATimestamp() % duration) / duration;
        for (int i = 0; i < data.getLength(); i++) data.setHSV(i, (int)(timeFraction * 255), 255, 255);
    }
}
