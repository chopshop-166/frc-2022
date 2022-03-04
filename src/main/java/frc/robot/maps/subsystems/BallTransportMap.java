package frc.robot.maps.subsystems;

import java.util.function.BooleanSupplier;

import com.chopshop166.chopshoplib.digital.MockDigitalInput;
import com.chopshop166.chopshoplib.motors.SmartMotorController;
import com.chopshop166.chopshoplib.sensors.IColorSensor;
import com.chopshop166.chopshoplib.sensors.MockColorSensor;

public class BallTransportMap {
    private final SmartMotorController bottomMotor;
    private final SmartMotorController topMotor;
    private final IColorSensor colorSensor;
    private final BooleanSupplier laserSwitch;

    public BallTransportMap() {
        this.bottomMotor = new SmartMotorController();

        this.topMotor = new SmartMotorController();

        this.colorSensor = new MockColorSensor();

        this.laserSwitch = new MockDigitalInput();
    }

    public BallTransportMap(final SmartMotorController bottomMotor, final SmartMotorController topMotor,
            final IColorSensor colorSensor, final BooleanSupplier laserSwitch) {

        this.bottomMotor = bottomMotor;

        this.topMotor = topMotor;

        this.colorSensor = colorSensor;

        this.laserSwitch = laserSwitch;
    }

    public SmartMotorController getBottomMotor() {
        return bottomMotor;
    }

    public SmartMotorController getTopMotor() {
        return topMotor;
    }

    public IColorSensor getColorSensor() {
        return colorSensor;
    }

    public BooleanSupplier getLaserSwitch() {
        return laserSwitch;
    }
}