package frc.robot.maps;

import com.chopshop166.chopshoplib.maps.RobotMapFor;
import com.chopshop166.chopshoplib.motors.PIDSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.DigitalInput;

@RobotMapFor("Gaston")
public class GastonMap extends RobotMap {
    // These are outsied the methods to prevent resource leaks
    // climber stuff
    private final DigitalInput leftUpperLimit = new DigitalInput(0);
    private final DigitalInput leftLowerLimit = new DigitalInput(1);
    private final DigitalInput rightUpperLimit = new DigitalInput(2);
    private final DigitalInput rightLowerLimit = new DigitalInput(3);

    private final PIDSparkMax leftMotor = new PIDSparkMax(0, MotorType.kBrushless);
    private final PIDSparkMax rightMotor = new PIDSparkMax(1, MotorType.kBrushless);

    // intake stuff
    private final DigitalInput outsideLimit = new DigitalInput(4);
    private final DigitalInput insideLimit = new DigitalInput(5);

    private final PIDSparkMax deploymentMotor = new PIDSparkMax(2, MotorType.kBrushless);
    private final PIDSparkMax rollerMotor = new PIDSparkMax(3, MotorType.kBrushless);
    private final SparkMaxPIDController deploymentPidController = deploymentMotor.getPidController();

    public IntakeMap getIntakeMap() {
        // PID coefficients
        // initializes relative encoder and pid controller, we don't need the encoder rn

        // private RelativeEncoder deploymentEncoder =
        // deploymentMotor.getEncoder().getRaw();

        double P = 0;
        double I = 0;
        double D = 0;
        double IZone = 0;
        double maxOutput = 1;
        double minOutput = -1;

        // Smart motion coefficients
        // max velocity is in rpm!!

        double smartMotionMaxVel = 30;
        double smartMotionOutputVel = 0;
        double smartMotionMaxAccel = 600;

        // TODO pid/smart motion coefficients for intake

        deploymentPidController.setP(P);
        deploymentPidController.setI(I);
        deploymentPidController.setD(D);
        deploymentPidController.setIZone(IZone);
        deploymentPidController.setOutputRange(minOutput, maxOutput);

        deploymentPidController.setSmartMotionMaxVelocity(smartMotionMaxVel, 0);
        deploymentPidController.setSmartMotionMinOutputVelocity(smartMotionOutputVel, 0);
        deploymentPidController.setSmartMotionMaxAccel(smartMotionMaxAccel, 0);

        IntakeMap map = new IntakeMap(rollerMotor, deploymentMotor, outsideLimit::get, insideLimit::get);
        return map;

    }

    public TelescopeMap getLeftTelescopeMap() {
        return new TelescopeMap(leftMotor, leftUpperLimit::get, leftLowerLimit::get);
    }

    public TelescopeMap getRightTelescopeMap() {
        return new TelescopeMap(rightMotor, rightUpperLimit::get, rightLowerLimit::get);
    }
}
