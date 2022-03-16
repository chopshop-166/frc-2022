package frc.robot.maps;

import com.chopshop166.chopshoplib.digital.WDigitalInput;
import com.chopshop166.chopshoplib.drive.SDSSwerveModule;
import com.chopshop166.chopshoplib.drive.SwerveDriveMap;
import com.chopshop166.chopshoplib.maps.RobotMapFor;
import com.chopshop166.chopshoplib.motors.PIDSparkMax;
import com.chopshop166.chopshoplib.sensors.PigeonGyro;
import com.chopshop166.chopshoplib.sensors.REVColorSensor;
import com.chopshop166.chopshoplib.sensors.WEncoder;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix.sensors.PigeonIMU.PigeonState;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.I2C.Port;
import frc.robot.maps.subsystems.BallTransportMap;
import frc.robot.maps.subsystems.ClimberMap;
import frc.robot.maps.subsystems.IntakeMap;
import frc.robot.maps.subsystems.LedMap;
import frc.robot.maps.subsystems.ShooterMap;

@RobotMapFor("00:80:2F:17:62:25")
public class ValkyrieMap extends RobotMap {
    final int CLIMBER_EXTEND_LIMIT = 50;
    final int CLIMBER_ROTATE_LIMIT = 50;

    @Override
    public SwerveDriveMap getSwerveDriveMap() {
        // Value taken from CAD as offset from center of module base pulley to center
        // of the robot
        final double MODULE_OFFSET_XY = 0.314325;

        // All Distances are in Meters
        // Front Left Module
        final CANCoder encoderFL = new CANCoder(1);
        encoderFL.configMagnetOffset(180 + 74.119);
        encoderFL.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
        final SDSSwerveModule frontLeft = new SDSSwerveModule(new Translation2d(MODULE_OFFSET_XY, MODULE_OFFSET_XY),
                encoderFL, new PIDSparkMax(2, MotorType.kBrushless), new PIDSparkMax(1,
                        MotorType.kBrushless),
                SDSSwerveModule.MK4_V2);

        // Front Right Module
        final CANCoder encoderFR = new CANCoder(2);
        encoderFR.configMagnetOffset(-304.189 + 180);
        encoderFR.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
        final SDSSwerveModule frontRight = new SDSSwerveModule(new Translation2d(MODULE_OFFSET_XY, -MODULE_OFFSET_XY),
                encoderFR, new PIDSparkMax(4, MotorType.kBrushless), new PIDSparkMax(3,
                        MotorType.kBrushless),
                SDSSwerveModule.MK4_V2);

        // Rear Left Module
        final CANCoder encoderRL = new CANCoder(3);
        encoderRL.configMagnetOffset(-298.213);
        encoderRL.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
        final SDSSwerveModule rearLeft = new SDSSwerveModule(new Translation2d(-MODULE_OFFSET_XY, MODULE_OFFSET_XY),
                encoderRL, new PIDSparkMax(6, MotorType.kBrushless), new PIDSparkMax(5,
                        MotorType.kBrushless),
                SDSSwerveModule.MK4_V2);

        // Rear Right Module
        final CANCoder encoderRR = new CANCoder(4);
        encoderRR.configMagnetOffset(-168.223 + 180);
        encoderRR.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
        final SDSSwerveModule rearRight = new SDSSwerveModule(new Translation2d(-MODULE_OFFSET_XY, -MODULE_OFFSET_XY),
                encoderRR, new PIDSparkMax(8, MotorType.kBrushless), new PIDSparkMax(7,
                        MotorType.kBrushless),
                SDSSwerveModule.MK4_V2);

        final double maxDriveSpeedMetersPerSecond = Units.feetToMeters(10);

        final double maxRotationRadianPerSecond = Math.PI;

        // final Gyro gyro = new PigeonGyro(new PigeonIMU(5));
        final PigeonGyro pigeonGyro = new PigeonGyro(new PigeonIMU(0));

        return new SwerveDriveMap(frontLeft, frontRight, rearLeft, rearRight,
                maxDriveSpeedMetersPerSecond,
                maxRotationRadianPerSecond, pigeonGyro, () -> pigeonGyro.getRaw().getState() == PigeonState.Ready);
    }

    @Override
    public ShooterMap getShooterMap() {
        final PIDSparkMax motor = new PIDSparkMax(16, MotorType.kBrushless);
        final PIDSparkMax follower = new PIDSparkMax(15, MotorType.kBrushless);

        final WEncoder encoder = new WEncoder(1, 2, true, EncodingType.k1X);

        final PIDController pid = new PIDController(0.82212 / 4.0, 0, 0);

        SimpleMotorFeedforward ff = new SimpleMotorFeedforward(0.17243, 0.49992 /
                4.0, 0.6289);

        // Ks: -0.17243
        // Kv: 0.49992
        // Ka: 0.6289

        encoder.setDistancePerPulse(1.0 / 2048.0);

        motor.getMotorController().setIdleMode(IdleMode.kCoast);
        follower.getMotorController().setIdleMode(IdleMode.kCoast);
        // Kp : 0.82212
        motor.setControlType(CANSparkMax.ControlType.kVoltage);
        follower.getMotorController().follow(motor.getMotorController(), true);

        return new ShooterMap(motor, encoder, ff, pid);
    }

    @Override
    public IntakeMap getIntakeMap() {
        // Current limit in amps
        final int CURRENT_LIMIT = 40;

        final PIDSparkMax deploymentMotor = new PIDSparkMax(11,
                MotorType.kBrushless);
        final PIDSparkMax deploymentFollower = new PIDSparkMax(12,
                MotorType.kBrushless);
        final PIDSparkMax rollerMotor = new PIDSparkMax(13, MotorType.kBrushless);
        CANSparkMax deploymentController = deploymentMotor.getMotorController();
        CANSparkMax followerController = deploymentFollower.getMotorController();

        // Use current as a validator along with setting a current limit
        // on the motor controllers

        deploymentMotor.validateCurrent(CURRENT_LIMIT);

        deploymentFollower.getMotorController().follow(deploymentMotor.getMotorController(),
                true);
        deploymentMotor.getMotorController().setSmartCurrentLimit(CURRENT_LIMIT);
        deploymentFollower.getMotorController().setSmartCurrentLimit(CURRENT_LIMIT);
        rollerMotor.getMotorController().setInverted(true);

        return new IntakeMap(deploymentMotor, rollerMotor, deploymentController::getOutputCurrent,
                followerController::getOutputCurrent);

    }

    @Override
    public BallTransportMap getBallTransportMap() {
        final PIDSparkMax topMotor = new PIDSparkMax(14, MotorType.kBrushless);
        final PIDSparkMax bottomMotor = new PIDSparkMax(17, MotorType.kBrushless);

        final REVColorSensor colorSensor = new REVColorSensor(Port.kMXP);

        final WDigitalInput laserSwitch = new WDigitalInput(0);

        return new BallTransportMap(bottomMotor, topMotor, colorSensor, laserSwitch);
    }

    @Override
    public ClimberMap getLeftClimberMap() {
        // The current limit for the climber's motors in amps

        final PIDSparkMax extendMotor = new PIDSparkMax(9, MotorType.kBrushless);

        final PIDSparkMax rotateMotor = new PIDSparkMax(18, MotorType.kBrushless);

        final CANSparkMax extendController = extendMotor.getMotorController();
        final CANSparkMax rotateController = rotateMotor.getMotorController();

        extendMotor.getMotorController().setIdleMode(IdleMode.kBrake);
        rotateMotor.getMotorController().setIdleMode(IdleMode.kBrake);
        extendMotor.getMotorController().setInverted(false);
        rotateMotor.getMotorController().setInverted(false);
        // Setting the current limits on both the validators and motor controllers
        extendMotor.validateCurrent(CLIMBER_EXTEND_LIMIT);
        extendMotor.getMotorController().setSmartCurrentLimit(CLIMBER_EXTEND_LIMIT);
        rotateMotor.validateCurrent(CLIMBER_ROTATE_LIMIT);
        rotateMotor.getMotorController().setSmartCurrentLimit(CLIMBER_ROTATE_LIMIT);

        return new ClimberMap(extendMotor, rotateMotor, extendController::getOutputCurrent,
                rotateController::getOutputCurrent);
    }

    @Override
    public ClimberMap getRightClimberMap() {
        // The current limit for the climber's motors in amps

        final PIDSparkMax extendMotor = new PIDSparkMax(10, MotorType.kBrushless);

        final PIDSparkMax rotateMotor = new PIDSparkMax(19, MotorType.kBrushless);

        final CANSparkMax extendController = extendMotor.getMotorController();
        final CANSparkMax rotateController = rotateMotor.getMotorController();

        extendMotor.getMotorController().setIdleMode(IdleMode.kBrake);
        rotateMotor.getMotorController().setIdleMode(IdleMode.kBrake);
        extendMotor.getMotorController().setInverted(true);
        rotateMotor.getMotorController().setInverted(true);

        // Setting the current limits on both the validators and motor controllers
        extendMotor.validateCurrent(CLIMBER_EXTEND_LIMIT);
        extendMotor.getMotorController().setSmartCurrentLimit(CLIMBER_EXTEND_LIMIT);
        rotateMotor.validateCurrent(CLIMBER_ROTATE_LIMIT);
        rotateMotor.getMotorController().setSmartCurrentLimit(CLIMBER_ROTATE_LIMIT);

        return new ClimberMap(extendMotor, rotateMotor, extendController::getOutputCurrent,
                rotateController::getOutputCurrent);
    }

    @Override
    public LedMap getLedMap() {
        AddressableLED led = new AddressableLED(0);
        // Best if this is a multiple of 10
        AddressableLEDBuffer ledBuffer = new AddressableLEDBuffer(30);

        return new LedMap(led, ledBuffer);
    }
}
