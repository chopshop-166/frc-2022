package frc.robot.maps;

import com.chopshop166.chopshoplib.maps.RobotMapFor;
import com.chopshop166.chopshoplib.motors.PIDSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;

@RobotMapFor("Gaston")
public class GastonMap extends RobotMap {
    private final DigitalInput outsideLimit = new DigitalInput(4);
    private final DigitalInput insideLimit = new DigitalInput(5);

    // makes motor controller object
    private PIDSparkMax deploymentMotor = new PIDSparkMax(2, MotorType.kBrushless);

    private RelativeEncoder deploymentEncoder = deploymentMotor.getEncoder().getRaw();
    private final SparkMaxPIDController deploymentPidController = deploymentMotor.getPidController();

    // PID coefficients

    public IntakeMap getIntakeMap() {

        IntakeMap map = new IntakeMap(new PIDSparkMax(3, MotorType.kBrushless),
                deploymentMotor, outsideLimit::get, insideLimit::get);
        return map;

    }
}