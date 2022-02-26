package frc.robot.maps;

import com.chopshop166.chopshoplib.drive.SDSSwerveModule;
import com.chopshop166.chopshoplib.motors.PIDControlType;
import com.chopshop166.chopshoplib.maps.RobotMapFor;
import com.chopshop166.chopshoplib.motors.PIDSparkMax;
import com.chopshop166.chopshoplib.sensors.IEncoder;
import com.chopshop166.chopshoplib.sensors.PigeonGyro;
import com.chopshop166.chopshoplib.sensors.WEncoder;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.interfaces.Gyro;

@RobotMapFor("00:80:2F:17:62:25")
public class GastonMap extends RobotMap {
    final int CLIMBER_EXTEND_LIMIT = 20;
    final int CLIMBER_ROTATE_LIMIT = 20;

    @Override
    public SwerveDriveMap getSwerveDriveMap() {
        // Value taken from CAD as offset from center of module base pulley to center of
        // robot
        final double MODULE_OFFSET_XY = 0.314325;

        // All Distances are in Meters
        // Front Left Module
        final CANCoder encoderFL = new CANCoder(1);
        encoderFL.configMagnetOffset(-195.381);
        encoderFL.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
        final SDSSwerveModule frontLeft = new SDSSwerveModule(new Translation2d(MODULE_OFFSET_XY, MODULE_OFFSET_XY),
                encoderFL, new PIDSparkMax(2, MotorType.kBrushless), new PIDSparkMax(1, MotorType.kBrushless),
                SDSSwerveModule.MK4_V2);

        // Front Right Module
        final CANCoder encoderFR = new CANCoder(2);
        encoderFR.configMagnetOffset(-304.189 + 180);
        encoderFR.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
        final SDSSwerveModule frontRight = new SDSSwerveModule(new Translation2d(MODULE_OFFSET_XY, -MODULE_OFFSET_XY),
                encoderFR, new PIDSparkMax(4, MotorType.kBrushless), new PIDSparkMax(3, MotorType.kBrushless),
                SDSSwerveModule.MK4_V2);

        // Rear Left Module
        final CANCoder encoderRL = new CANCoder(3);
        encoderRL.configMagnetOffset(-298.213);
        encoderRL.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
        final SDSSwerveModule rearLeft = new SDSSwerveModule(new Translation2d(-MODULE_OFFSET_XY, MODULE_OFFSET_XY),
                encoderRL, new PIDSparkMax(6, MotorType.kBrushless), new PIDSparkMax(5, MotorType.kBrushless),
                SDSSwerveModule.MK4_V2);

        // Rear Right Module
        final CANCoder encoderRR = new CANCoder(4);
        encoderRR.configMagnetOffset(-168.223 + 180);
        encoderRR.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
        final SDSSwerveModule rearRight = new SDSSwerveModule(new Translation2d(-MODULE_OFFSET_XY, -MODULE_OFFSET_XY),
                encoderRR, new PIDSparkMax(8, MotorType.kBrushless), new PIDSparkMax(7, MotorType.kBrushless),
                SDSSwerveModule.MK4_V2);

        final double maxDriveSpeedMetersPerSecond = Units.feetToMeters(10);

        final double maxRotationRadianPerSecond = Math.PI;

        // final Gyro gyro = new PigeonGyro(new PigeonIMU(5));
        final Gyro pigeonGyro = new PigeonGyro(new PigeonIMU(0));

        return new SwerveDriveMap(frontLeft, frontRight, rearLeft, rearRight, maxDriveSpeedMetersPerSecond,
                maxRotationRadianPerSecond, pigeonGyro);
    }

    @Override
    public ShooterMap getShooterMap() {
        // Determine encoder pins
        final IEncoder encoder = new WEncoder(0, 1);
        final PIDSparkMax motor = new PIDSparkMax(15, MotorType.kBrushless);
        final PIDSparkMax follower = new PIDSparkMax(16, MotorType.kBrushless);

        motor.setControlType(PIDControlType.Velocity);
        follower.setControlType(PIDControlType.Velocity);
        follower.getMotorController().follow(motor.getMotorController(), true);

        return new ShooterMap(motor, encoder);
    }

    @Override
    public IntakeMap getIntakeMap() {
        // Current limit in amps
        final int CURRENT_LIMIT = 30;

        final PIDSparkMax deploymentMotor = new PIDSparkMax(11, MotorType.kBrushless);
        final PIDSparkMax deploymentFollower = new PIDSparkMax(12, MotorType.kBrushless);
        final PIDSparkMax rollerMotor = new PIDSparkMax(13, MotorType.kBrushless);

        // Use current as a validator along with setting a current limit
        // on the motor controllers

        deploymentMotor.validateCurrent(CURRENT_LIMIT);

        deploymentFollower.getMotorController().follow(deploymentMotor.getMotorController(), true);
        deploymentMotor.getMotorController().setSmartCurrentLimit(CURRENT_LIMIT);
        deploymentFollower.getMotorController().setSmartCurrentLimit(CURRENT_LIMIT);
        rollerMotor.getMotorController().setInverted(true);

        return new IntakeMap(deploymentMotor, rollerMotor);

    }

    @Override
    public ClimberMap getLeftClimberMap() {
        // The current limit for the climber's motors in amps

        final PIDSparkMax extendMotor = new PIDSparkMax(9, MotorType.kBrushless);

        final PIDSparkMax rotateMotor = new PIDSparkMax(18, MotorType.kBrushless);

        // Setting the current limits on both the validators and motor controllers
        extendMotor.validateCurrent(CLIMBER_EXTEND_LIMIT);
        extendMotor.getMotorController().setSmartCurrentLimit(CLIMBER_EXTEND_LIMIT);
        rotateMotor.validateCurrent(CLIMBER_ROTATE_LIMIT);
        rotateMotor.getMotorController().setSmartCurrentLimit(CLIMBER_ROTATE_LIMIT);

        return new ClimberMap(extendMotor, rotateMotor);
    }

    @Override
    public ClimberMap getRightClimberMap() {
        // The current limit for the climber's motors in amps
        final PIDSparkMax extendMotor = new PIDSparkMax(10, MotorType.kBrushless);

        final PIDSparkMax rotateMotor = new PIDSparkMax(19, MotorType.kBrushless);

        // Setting the current limits on both the validators and motor controllers
        extendMotor.validateCurrent(CLIMBER_EXTEND_LIMIT);
        extendMotor.getMotorController().setSmartCurrentLimit(CLIMBER_EXTEND_LIMIT);
        rotateMotor.validateCurrent(CLIMBER_ROTATE_LIMIT);
        rotateMotor.getMotorController().setSmartCurrentLimit(CLIMBER_ROTATE_LIMIT);

        return new ClimberMap(extendMotor, rotateMotor);
    }
}
