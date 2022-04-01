package frc.robot.subsystems;

import com.chopshop166.chopshoplib.commands.SmartSubsystemBase;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.maps.subsystems.LedMap;
import frc.robot.util.LightAnimation;

public class Led extends SmartSubsystemBase {

    private final AddressableLED led;
    private final AddressableLEDBuffer ledBuffer;
    private final SerialPort serialPort;

    private int timer = 0;

    public Led(final LedMap map) {
        led = map.getLed();
        serialPort = map.getSerialPort();
        ledBuffer = map.getLedBuffer();
        led.setLength(ledBuffer.getLength());
        led.start();
    }

    public CommandBase animate(LightAnimation animation, double brightness) {
        return cmd("Animate").onExecute(() -> {
            for (int i = 0; i < ledBuffer.getLength(); i++) {
                int bufLen = (ledBuffer.getLength() / 10);
                Color c = animation.getColor(0,
                        i / bufLen + (timer / 10));

                ledBuffer.setLED(i, new Color(
                        c.red * brightness,
                        c.green * brightness,
                        c.blue * brightness));
            }
            led.setData(ledBuffer);
            timer++;
        }).runsWhenDisabled(true);

    }

    public CommandBase serialPortSend() {
        return cmd("Serial Connection").onExecute(() -> {
            Alliance currentAlliance = DriverStation.getAlliance();
            byte[] sendData = new byte[1];
            int output;
            switch (currentAlliance) {
                case Red:
                    sendData[0] = 'r';
                    break;
                case Blue:
                    sendData[0] = 'b';
                    break;
                default:
                    sendData[0] = 'n';
                    break;
            }
            output = serialPort.write(sendData, 1);
            System.out.println(sendData[0] + "  " + output);
        }).runsWhenDisabled(true).runsUntil(() -> true);
    }

    @Override
    public void periodic() {

    }

    @Override
    public void safeState() {
    }
}