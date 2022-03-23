package frc.robot.maps.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.SerialPort;

public class LedMap {
    private final AddressableLED led;
    private final AddressableLEDBuffer ledBuffer;
    private final SerialPort serialPort;

    public LedMap(AddressableLED led, AddressableLEDBuffer ledBuffer, SerialPort serialPort) {
        this.led = led;
        this.ledBuffer = ledBuffer;
        this.serialPort = serialPort;
    }

    public LedMap() {
        this(new AddressableLED(0), new AddressableLEDBuffer(0), new SerialPort(9600, SerialPort.Port.kUSB));
    }

    public AddressableLED getLed() {
        return led;
    }

    public AddressableLEDBuffer getLedBuffer() {
        return ledBuffer;
    }

    public SerialPort getSerialPort() {
        return serialPort;
    }

}