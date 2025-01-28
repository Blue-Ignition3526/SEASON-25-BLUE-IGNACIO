package lib.BlueShift.led.framework;

public abstract class HyperLEDAnimation {
    int r;
    int g;
    int b;

    public void provider(HyperAddressableLEDBuffer data) {};

    public HyperLEDAnimation setRGB(int r, int g, int b) {
        this.r = r;
        this.g = g;
        this.b = b;
        return this;
    };
}
