package frc.robot.maps;

import com.chopshop166.chopshoplib.drive.SwerveDriveMap;
import com.chopshop166.chopshoplib.maps.RobotMapFor;

import frc.robot.maps.subsystems.BallTransportMap;
import frc.robot.maps.subsystems.ClimberMap;
import frc.robot.maps.subsystems.IntakeMap;
import frc.robot.maps.subsystems.ShooterMap;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.interfaces.Gyro;

@RobotMapFor("Default")
public class RobotMap {

    public SwerveDriveMap getSwerveDriveMap() {
        return new SwerveDriveMap();
    }

    public BallTransportMap getBallTransportMap() {
        return new BallTransportMap();
    }

    public IntakeMap getIntakeMap() {
        return new IntakeMap();
    }

    public ShooterMap getShooterMap() {
        return new ShooterMap();
    }

    public ClimberMap getLeftClimberMap() {
        return new ClimberMap();
    }

    public ClimberMap getRightClimberMap() {
        return new ClimberMap();
    }

    public static class LedMap {
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

    public LedMap getLedMap() {
        return new LedMap();
    }
}
