package frc.robot.maps;

import com.chopshop166.chopshoplib.maps.RobotMapFor;
import com.chopshop166.chopshoplib.motors.SmartMotorController;

@RobotMapFor("Gaston")
public class GastonMap extends RobotMap {
    public ShooterMap getShooterMap() {
        ShooterMap map = new ShooterMap(new SmartMotorController(), new SmartMotorController(),
                new SmartMotorController(), new SmartMotorController());
        return map;
    }
}
