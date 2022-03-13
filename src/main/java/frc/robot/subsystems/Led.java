package frc.robot.subsystems;

import com.chopshop166.chopshoplib.commands.SmartSubsystemBase;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.maps.subsystems.LedMap;
import frc.robot.util.LightAnimation;

public class Led extends SmartSubsystemBase {

    private final AddressableLED led;
    private final AddressableLEDBuffer ledBuffer;

    private int frame = 0;
    private int timer = 0;

    public Led(final LedMap map) {
        led = map.getLed();
        ledBuffer = map.getLedBuffer();
        led.setLength(ledBuffer.getLength());
        led.start();
    }

    public CommandBase animate(LightAnimation animation, double brightness) {
        return running("Animate", () -> {
            for (int i = 0; i < ledBuffer.getLength(); i++) {
                int bufLen = (ledBuffer.getLength() / 10);
                Color c = animation.getColor(timer,
                        i / bufLen, (i % bufLen) / ((float) bufLen));
                ledBuffer.setLED(i, new Color(
                        c.red * brightness,
                        c.green * brightness,
                        c.blue * brightness));
            }
            led.setData(ledBuffer);
            timer++;
        });

    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    @Override
    public void safeState() {
    }
}