package lib.BlueShift.led.framework;

import java.util.function.Consumer;

public class HyperAddressableLEDSegment {
    HyperAddressableLEDBuffer buffer;
    private final int length;
    private int offset = 0;
    private Consumer<HyperAddressableLEDBuffer> defaultAnimation;
    private boolean isPlayingdefaultAnimation = false;
    private Consumer<HyperAddressableLEDBuffer> currentAnimation;
    private boolean isPlayingCurrentAnimation = false;

    public HyperAddressableLEDSegment(int length) {
        this.length = length;
        buffer = new HyperAddressableLEDBuffer(length);
    }

    public HyperAddressableLEDSegment(HyperAddressableLEDBuffer buffer) {
        this.buffer = buffer;
        this.length = buffer.getLength();
    }

    public int getLength() {
        return length;
    }

    public int getOffset() {
        return offset;
    }

    void setOffset(int offset) {
        this.offset = offset;
    }

    public HyperAddressableLEDSegment setAnimation(Consumer<HyperAddressableLEDBuffer> animation) {
        this.currentAnimation = animation;
        this.isPlayingCurrentAnimation = true;
        return this;
    }

    public HyperAddressableLEDSegment stopAnimation() {
        this.isPlayingCurrentAnimation = false;
        return this;
    }

    public HyperAddressableLEDSegment resumeAnimation() {
        this.isPlayingCurrentAnimation = true;
        this.isPlayingdefaultAnimation = false;
        return this;
    }

    public HyperAddressableLEDSegment setDefaultAnimation(Consumer<HyperAddressableLEDBuffer> defaultAnimation) {
        this.defaultAnimation = defaultAnimation;
        this.isPlayingdefaultAnimation = true;
        return this;
    }

    public HyperAddressableLEDSegment stopDefaultAnimation() {
        this.isPlayingdefaultAnimation = false;
        return this;
    }

    public HyperAddressableLEDSegment resumeDefaultAnimation() {
        this.isPlayingdefaultAnimation = true;
        this.isPlayingCurrentAnimation = false;
        return this;
    }

    public HyperAddressableLEDSegment setPixel(int index, int r, int g, int b) {
        buffer.setRGB(index + offset, r, g, b);
        return this;
    }

    public HyperAddressableLEDSegment setPixel(int index, int r, int g, int b, int brightness) {
        buffer.setRGB(index + offset, r, g, b, brightness);
        return this;
    }

    public HyperAddressableLEDSegment setRGB(int r, int g, int b) {
        for (int i = 0; i < length; i++) buffer.setRGB(i + offset, r, g, b);
        return this;
    }

    public void update() {
        if (this.currentAnimation != null && isPlayingCurrentAnimation) this.currentAnimation.accept(buffer);
        else if (this.defaultAnimation != null && isPlayingdefaultAnimation) this.defaultAnimation.accept(buffer);
    }
}
