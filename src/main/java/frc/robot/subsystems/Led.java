package frc.robot.subsystems;

import java.util.function.Supplier;

import com.chopshop166.chopshoplib.SampleBuffer;
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

    public enum BallColor {
        RED(Color.kFirstRed), BLUE(Color.kFirstBlue), NONE(new Color(0, 0, 0));

        private final Color color;

        private BallColor(Color color) {
            this.color = color;
        }

        public Color get() {
            return color;
        }
    }

    public Led(final LedMap map) {
        led = map.getLed();
        ledBuffer = map.getLedBuffer();
        led.setLength(ledBuffer.getLength());
        led.start();
    }

    private void runAnimation(LightAnimation animation, double brightness) {
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
    }

    public CommandBase animate(LightAnimation animation, double brightness) {
        return running("Animate", () -> {
            runAnimation(animation, brightness);
            timer++;
        });
    }

    private BallColor matchColor(Color c) {
        if (c.red > c.green && c.red > c.blue) {
            return BallColor.RED;
        }
        if (c.blue > c.green && c.blue > c.red) {
            return BallColor.BLUE;
        }
        return BallColor.NONE;
    }

    public CommandBase showBallColors(Supplier<SampleBuffer<Color>> colorBuffer, LightAnimation defaultAnimation,
            double brightness) {
        return running("Show Ball Colors", () -> {
            SampleBuffer<Color> colors = colorBuffer.get();
            if (colors.size() == 0) {
                runAnimation(defaultAnimation, brightness);
            } else {
                Color bottomColor = matchColor(colors.peekLast()).get();
                Color topColor = (colors.size() == 1) ? (BallColor.NONE.get()) : (matchColor(colors.peekFirst()).get());

                double sin = (Math.sin(timer / 10.0) + 1.0) / 2.0;
                for (int i = 0; i < ledBuffer.getLength(); i++) {
                    boolean isBottom = i < ledBuffer.getLength() / 2;
                    Color c = isBottom ? bottomColor : topColor;
                    double flash = isBottom ? sin : 1.0 - sin;

                    ledBuffer.setLED(i, new Color(
                            c.red * flash * brightness,
                            c.green * flash * brightness,
                            c.blue * flash * brightness));

                }
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