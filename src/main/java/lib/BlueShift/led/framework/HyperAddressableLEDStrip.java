package lib.BlueShift.led.framework;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class HyperAddressableLEDStrip extends SubsystemBase {
    public HyperAddressableLEDSegment[] segments;
    public HyperAddressableLEDBuffer buffer;
    public HyperAddressableLED leds;
    public int port;

    public HyperAddressableLEDStrip(int port, HyperAddressableLEDSegment... segments) {
        this.port = port;
        this.segments = segments;
        int length = 0;
        for (HyperAddressableLEDSegment segment : segments) {
            segment.setOffset(length);
            length += segment.getLength();
        }
        this.buffer = new HyperAddressableLEDBuffer(length);
        this.leds = new HyperAddressableLED(port);
        this.leds.setLength(length);
        this.leds.setData(buffer);
        this.leds.start();
    }

    @Override
    public void periodic() {
        for (HyperAddressableLEDSegment segment : segments) {
            segment.update();
            segment.buffer.copy(buffer, segment.getOffset());
        }
        leds.setData(buffer);
    }
}
