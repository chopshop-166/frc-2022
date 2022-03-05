package frc.robot.subsystems;

import com.chopshop166.chopshoplib.commands.SmartSubsystemBase;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.maps.RobotMap.LedMap;
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
            if (timer % 5 == 0) {
                for (int i = 0; i < ledBuffer.getLength(); i++) {
                    Color c = animation.getColor(frame,
                            i / (ledBuffer.getLength() / 10));
                    ledBuffer.setLED(i, new Color(
                            c.red * brightness,
                            c.green * brightness,
                            c.blue * brightness));
                }
                led.setData(ledBuffer);
                frame++;
            }
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