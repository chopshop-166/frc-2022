package frc.robot.subsystems;

import com.chopshop166.chopshoplib.commands.SmartSubsystemBase;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import frc.robot.maps.RobotMap.LedMap;

public class Led extends SmartSubsystemBase {

    private final AddressableLED led;
    private final AddressableLEDBuffer ledBuffer;

    public Led(final LedMap map) {
        led = map.getLed();
        ledBuffer = map.getLedBuffer();
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    @Override
    public void safeState() {
    }
}