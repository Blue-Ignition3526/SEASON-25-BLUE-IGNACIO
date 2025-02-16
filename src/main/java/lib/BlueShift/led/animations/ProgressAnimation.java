package lib.BlueShift.led.animations;

import lib.BlueShift.led.framework.HyperAddressableLEDBuffer;
import lib.BlueShift.led.framework.HyperLEDAnimation;

public class ProgressAnimation extends HyperLEDAnimation {
    private int r;
    private int g;
    private int b;
    private double progress = 0;

    public ProgressAnimation(int r, int g, int b) {
        this.r = r;
        this.g = g;
        this.b = b;
    }

    public void setProgress(double progress) {
        this.progress = progress;
    }

    public void provider(HyperAddressableLEDBuffer data) {
        int length = data.getLength();
        int progressLength = (int)(length * progress);

        for (int i = 0; i < length; i++) {
            if (i < progressLength) {
                if (this.progress == 1) data.setRGB(i, r, g, b, 255);
                else data.setRGB(i, r, g, b, 255 - (int)(255 * (progressLength - i) / (double)progressLength));
            } else data.setRGB(i, 0, 0, 0);
        }
    }
}
