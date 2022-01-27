package frc.robot.maps;

import com.chopshop166.chopshoplib.maps.RobotMapFor;
import com.chopshop166.chopshoplib.motors.PIDSparkMax;
import com.chopshop166.chopshoplib.motors.SmartMotorController;

import frc.robot.maps.RobotMap.ShooterMap;
import frc.robot.subsystems.Shooter;
import frc.robot.maps.*;

@RobotMapFor("Gaston")
public class GastonMap extends RobotMap {
    public ShooterMap getShooterMap() {
        ShooterMap map = new ShooterMap(new SmartMotorController(), new SmartMotorController(),
                new SmartMotorController(), new SmartMotorController());
        return map;
    }
}
