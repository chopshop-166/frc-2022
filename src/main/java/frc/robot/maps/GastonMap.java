package frc.robot.maps;

import com.chopshop166.chopshoplib.maps.RobotMapFor;
import com.chopshop166.chopshoplib.motors.PIDSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;

@RobotMapFor("Gaston")
public class GastonMap extends RobotMap {
    private final DigitalInput leftUpperLimit = new DigitalInput(0);
    private final DigitalInput leftLowerLimit = new DigitalInput(1);
    private final DigitalInput rightUpperLimit = new DigitalInput(2);
    private final DigitalInput rightLowerLimit = new DigitalInput(3);

    public TelescopeMap getLeftTelescopeMap() {
        TelescopeMap map = new TelescopeMap(new PIDSparkMax(0, MotorType.kBrushless), leftUpperLimit::get,
                leftLowerLimit::get);
        return map;
    }

    public TelescopeMap getRightTelescopeMap() {
        TelescopeMap map = new TelescopeMap(new PIDSparkMax(1, MotorType.kBrushless), rightUpperLimit::get,
                rightLowerLimit::get);
        return map;
    }
}