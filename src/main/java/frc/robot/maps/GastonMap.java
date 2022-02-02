package frc.robot.maps;

import com.chopshop166.chopshoplib.maps.RobotMapFor;
import com.chopshop166.chopshoplib.motors.PIDSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;

@RobotMapFor("Gaston")
public class GastonMap extends RobotMap {
    // These are outsied the methods to prevent resource leaks
    private final DigitalInput leftUpperLimit = new DigitalInput(0);
    private final DigitalInput leftLowerLimit = new DigitalInput(1);
    private final DigitalInput rightUpperLimit = new DigitalInput(2);
    private final DigitalInput rightLowerLimit = new DigitalInput(3);

    private final PIDSparkMax leftMotor = new PIDSparkMax(0, MotorType.kBrushless);
    private final PIDSparkMax rightMotor = new PIDSparkMax(1, MotorType.kBrushless);

    public TelescopeMap getLeftTelescopeMap() {
        TelescopeMap map = new TelescopeMap(leftMotor, leftUpperLimit::get, leftLowerLimit::get);
        return map;
    }

    public TelescopeMap getRightTelescopeMap() {
        TelescopeMap map = new TelescopeMap(rightMotor, rightUpperLimit::get, rightLowerLimit::get);
        return map;
    }
}