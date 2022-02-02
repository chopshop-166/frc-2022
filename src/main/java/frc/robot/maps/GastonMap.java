package frc.robot.maps;

import com.chopshop166.chopshoplib.maps.RobotMapFor;
import com.chopshop166.chopshoplib.motors.PIDSparkMax;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.DigitalInput;

@RobotMapFor("Gaston")
public class GastonMap extends RobotMap {
    private final DigitalInput outsideLimit = new DigitalInput(4);
    private final DigitalInput insideLimit = new DigitalInput(5);

    // initializes relative encoder and pid controller, we don't need the encoder rn

    private PIDSparkMax deploymentMotor = new PIDSparkMax(2, MotorType.kBrushless);

    // private RelativeEncoder deploymentEncoder =
    // deploymentMotor.getEncoder().getRaw();
    private final SparkMaxPIDController deploymentPidController = deploymentMotor.getPidController();

    public IntakeMap getIntakeMap() {
        // PID coefficients

        double P = 5e-5;
        double I = 1e-6;
        double D = 0;
        double IZone = 0;
        double feedForward = 0.000156;
        double maxOutput = 1;
        double minOutput = -1;

        // Smart motion coefficients
        // max velocity is in rpm!!

        double smartMotionMaxVel = 30;
        double smartMotionOutputVel = 0;
        double smartMotionMaxAccel = 600;

        // TODO pid/smart motion cooefficients for intake

        deploymentPidController.setP(P);
        deploymentPidController.setI(I);
        deploymentPidController.setD(D);
        deploymentPidController.setIZone(IZone);
        deploymentPidController.setFF(feedForward);
        deploymentPidController.setOutputRange(minOutput, maxOutput);

        deploymentPidController.setSmartMotionMaxVelocity(smartMotionMaxVel, 0);
        deploymentPidController.setSmartMotionMinOutputVelocity(smartMotionOutputVel, 0);
        deploymentPidController.setSmartMotionMaxAccel(smartMotionMaxAccel, 0);

        // the value here is in rotations
        deploymentPidController.setReference(0.25, CANSparkMax.ControlType.kSmartMotion);

        IntakeMap map = new IntakeMap(new PIDSparkMax(3, MotorType.kBrushless),
                deploymentMotor, outsideLimit::get, insideLimit::get);
        return map;

    }
}