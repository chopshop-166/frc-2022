package frc.robot.maps;

import com.chopshop166.chopshoplib.maps.RobotMapFor;
import com.chopshop166.chopshoplib.motors.PIDSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

@RobotMapFor("Gaston")
public class GastonMap extends RobotMap {
    private final DigitalInput outsideLimit = new DigitalInput(4);
    private final DigitalInput insideLimit = new DigitalInput(5);

    // makes motor controller object
    private PIDSparkMax deploymentMotor = new PIDSparkMax(2, MotorType.kBrushless);

    // initializes relative encoder and pid controller
    private RelativeEncoder deploymentEncoder = deploymentMotor.getEncoder().getRaw();
    private final SparkMaxPIDController deploymentPidController = deploymentMotor.getPidController();

    public IntakeMap getIntakeMap() {
        // PID coefficients

        // feel free to change these values as needed
        // they are example values

        deploymentPidController.setP(5e-5);
        deploymentPidController.setI(1e-6);
        deploymentPidController.setD(0);
        deploymentPidController.setIZone(0);
        deploymentPidController.setFF(0.000156);
        deploymentPidController.setOutputRange(-1, 1);

        // Smart motion coefficients
        // Max velocity is in rpm
        deploymentPidController.setSmartMotionMaxVelocity(2000, 0);
        deploymentPidController.setSmartMotionMinOutputVelocity(-1, 0);
        deploymentPidController.setSmartMotionMaxAccel(1500, 0);
        // deploymentPidController.setSmartMotionAllowedClosedLoopError(allowedErr,
        // slotID)

        // display PID coefficients on SmartDashboard
        /*
         * SmartDashboard.putNumber("P Gain", kP);
         * SmartDashboard.putNumber("I Gain", kI);
         * SmartDashboard.putNumber("D Gain", kD);
         * SmartDashboard.putNumber("I Zone", kIz);
         * SmartDashboard.putNumber("Feed Forward", kFF);
         * SmartDashboard.putNumber("Max Output", kMaxOutput);
         * SmartDashboard.putNumber("Min Output", kMinOutput);
         */

        IntakeMap map = new IntakeMap(new PIDSparkMax(3, MotorType.kBrushless),
                deploymentMotor, outsideLimit::get, insideLimit::get);
        return map;

    }
}