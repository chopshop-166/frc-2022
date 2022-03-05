package frc.robot.maps.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;

public class LedMap {
    private final AddressableLED led;
    private final AddressableLEDBuffer ledBuffer;

    public LedMap(AddressableLED led, AddressableLEDBuffer ledBuffer) {
        this.led = led;
        this.ledBuffer = ledBuffer;
    }

    public LedMap() {
        this(new AddressableLED(0), new AddressableLEDBuffer(0));
    }

    public AddressableLED getLed() {
        return led;
    }

    public AddressableLEDBuffer getLedBuffer() {
        return ledBuffer;
    }

}