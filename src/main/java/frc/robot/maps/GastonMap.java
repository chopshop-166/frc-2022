package frc.robot.maps;

import com.chopshop166.chopshoplib.drive.SDSSwerveModule;
import com.chopshop166.chopshoplib.motors.PIDControlType;
import com.chopshop166.chopshoplib.motors.PIDSparkMax;
import com.chopshop166.chopshoplib.sensors.PigeonGyro;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.interfaces.Gyro;

public class GastonMap extends RobotMap {

    private final double CLIMBER_CURRENT_LIMIT = 30.0; // The current limit for the climber's motors

    @Override
    public SwerveDriveMap getSwerveDriveMap() {
        // Value taken from CAD as offset from center of module base pulley to center of
        // robot
        final double MODULE_OFFSET_XY = 0.314325;

        // All Distances are in Meters
        // Front Left Module
        final CANCoder encoderFL = new CANCoder(1);
        encoderFL.configMagnetOffset(0); // TODO Get Magnet Offset
        encoderFL.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
        final SDSSwerveModule frontLeft = new SDSSwerveModule(new Translation2d(MODULE_OFFSET_XY, MODULE_OFFSET_XY),
                encoderFL, new PIDSparkMax(2, MotorType.kBrushless), new PIDSparkMax(1, MotorType.kBrushless),
                SDSSwerveModule.MK4_V2);

        // Front Right Module
        final CANCoder encoderFR = new CANCoder(2);
        encoderFR.configMagnetOffset(0); // TODO Get Magnet Offset
        encoderFR.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
        final SDSSwerveModule frontRight = new SDSSwerveModule(new Translation2d(MODULE_OFFSET_XY, -MODULE_OFFSET_XY),
                encoderFR, new PIDSparkMax(4, MotorType.kBrushless), new PIDSparkMax(3, MotorType.kBrushless),
                SDSSwerveModule.MK4_V2);

        // Rear Left Module
        final CANCoder encoderRL = new CANCoder(3);
        encoderRL.configMagnetOffset(0); // TODO Get Magnet Offset
        encoderRL.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
        final SDSSwerveModule rearLeft = new SDSSwerveModule(new Translation2d(-MODULE_OFFSET_XY, MODULE_OFFSET_XY),
                encoderRL, new PIDSparkMax(6, MotorType.kBrushless), new PIDSparkMax(5, MotorType.kBrushless),
                SDSSwerveModule.MK4_V2);

        // Rear Right Module
        final CANCoder encoderRR = new CANCoder(4);
        encoderRR.configMagnetOffset(0); // TODO Get Magnet Offset
        encoderRR.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
        final SDSSwerveModule rearRight = new SDSSwerveModule(new Translation2d(-MODULE_OFFSET_XY, -MODULE_OFFSET_XY),
                encoderRR, new PIDSparkMax(8, MotorType.kBrushless), new PIDSparkMax(7, MotorType.kBrushless),
                SDSSwerveModule.MK4_V2);

        final double maxDriveSpeedMetersPerSecond = Units.feetToMeters(10);

        final double maxRotationRadianPerSecond = Math.PI;

        final Gyro gyro = new PigeonGyro(new PigeonIMU(5));

        return new SwerveDriveMap(frontLeft, frontRight, rearLeft, rearRight, maxDriveSpeedMetersPerSecond,
                maxRotationRadianPerSecond, gyro);
    }

    public ShooterMap getShooterMap() {
        final PIDSparkMax shooterMoter = new PIDSparkMax(13, MotorType.kBrushless);
        final PIDSparkMax shooterLoadingMoter = new PIDSparkMax(14, MotorType.kBrushless);
        final double shootWheelRadius = 2.0; // Radius in inches

        // Convert to rotations per minute?
        final double velocityMultiplier = shootWheelRadius * 2 * Math.PI / (60 * 12);

        shooterMoter.setControlType(PIDControlType.Velocity);
        return new ShooterMap(shooterMoter, shooterLoadingMoter, shootWheelRadius, velocityMultiplier);
    }

    public IntakeMap getIntakeMap() {
        // PID coefficients
        // initializes relative encoder and pid controller, we don't need the encoder rn

        // private RelativeEncoder deploymentEncoder =
        // deploymentMotor.getEncoder().getRaw();
        final DigitalInput outsideLimit = new DigitalInput(5);
        final DigitalInput insideLimit = new DigitalInput(6);

        final PIDSparkMax deploymentMotor = new PIDSparkMax(11, MotorType.kBrushless);
        final PIDSparkMax rollerMotor = new PIDSparkMax(12, MotorType.kBrushless);
        final SparkMaxPIDController deploymentPidController = deploymentMotor.getPidController();

        double P = 0;
        double I = 0;
        double D = 0;
        double IZone = 0;
        double maxOutput = 1;
        double minOutput = -1;

        // TODO pid/smart motion coefficients for intake

        deploymentPidController.setP(P);
        deploymentPidController.setI(I);
        deploymentPidController.setD(D);
        deploymentPidController.setIZone(IZone);
        deploymentPidController.setOutputRange(minOutput, maxOutput);

        deploymentPidController.setSmartMotionMaxVelocity(30, 0);
        deploymentPidController.setSmartMotionMinOutputVelocity(0, 0);
        deploymentPidController.setSmartMotionMaxAccel(600, 0);

        return new IntakeMap(rollerMotor, deploymentMotor, outsideLimit::get, insideLimit::get);

    }

    public ClimberMap getLeftClimberMap() {
        final DigitalInput leftUpperLimit = new DigitalInput(1);
        final DigitalInput leftLowerLimit = new DigitalInput(2);
        final PIDSparkMax leftMotor = new PIDSparkMax(9, MotorType.kBrushless);
        leftMotor.validateCurrent(CLIMBER_CURRENT_LIMIT);
        return new ClimberMap(leftMotor, leftUpperLimit::get, leftLowerLimit::get);
    }

    public ClimberMap getRightClimberMap() {
        final DigitalInput rightUpperLimit = new DigitalInput(3);
        final DigitalInput rightLowerLimit = new DigitalInput(4);
        final PIDSparkMax rightMotor = new PIDSparkMax(10, MotorType.kBrushless);
        rightMotor.validateCurrent(CLIMBER_CURRENT_LIMIT);

        return new ClimberMap(rightMotor, rightUpperLimit::get, rightLowerLimit::get);
    }
}
