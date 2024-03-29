package frc.robot.maps;

import com.chopshop166.chopshoplib.digital.WDigitalInput;
import com.chopshop166.chopshoplib.drive.SDSSwerveModule;
import com.chopshop166.chopshoplib.drive.SwerveDriveMap;
import com.chopshop166.chopshoplib.maps.RobotMapFor;
import com.chopshop166.chopshoplib.motors.PIDSparkMax;
import com.chopshop166.chopshoplib.sensors.REVColorSensor;
import com.chopshop166.chopshoplib.sensors.WEncoder;
import com.chopshop166.chopshoplib.sensors.gyro.PigeonGyro;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.SerialPort;
import frc.robot.maps.subsystems.BallTransportMap;
import frc.robot.maps.subsystems.ClimberMap;
import frc.robot.maps.subsystems.IntakeMap;
import frc.robot.maps.subsystems.LedMap;
import frc.robot.maps.subsystems.ShooterMap;
import frc.robot.util.CurrentValidator;

@RobotMapFor("00:80:2F:17:62:25")
public class ValkyrieMap extends RobotMap {
    final int CLIMBER_EXTEND_LIMIT = 30;
    final int CLIMBER_ROTATE_LIMIT = 50;
    final PigeonGyro pigeonGyro = new PigeonGyro(new PigeonIMU(0));

    private final boolean swerveDriveOn = true;
    private final boolean ballTransportOn = true;
    private final boolean climberOn = true;
    private final boolean intakeOn = true;
    private final boolean ledOn = true;
    private final boolean shooterOn = true;

    @Override
    public SwerveDriveMap getSwerveDriveMap() {

        if (!swerveDriveOn) {
            return super.getSwerveDriveMap();
        }
        // Value taken from CAD as offset from center of module base pulley to center
        // of the robot
        final double MODULE_OFFSET_XY = 0.314325;

        final PIDSparkMax frontLeftSteer = new PIDSparkMax(2, MotorType.kBrushless);
        final PIDSparkMax frontRightSteer = new PIDSparkMax(4, MotorType.kBrushless);
        final PIDSparkMax rearLeftSteer = new PIDSparkMax(6, MotorType.kBrushless);
        final PIDSparkMax rearRightSteer = new PIDSparkMax(8, MotorType.kBrushless);

        frontLeftSteer.getMotorController().setPeriodicFramePeriod(PeriodicFrame.kStatus3, 1000);
        frontRightSteer.getMotorController().setPeriodicFramePeriod(PeriodicFrame.kStatus3, 1000);
        rearLeftSteer.getMotorController().setPeriodicFramePeriod(PeriodicFrame.kStatus3, 1000);
        frontRightSteer.getMotorController().setPeriodicFramePeriod(PeriodicFrame.kStatus3, 1000);

        // All Distances are in Meters
        // Front Left Module
        final CANCoder encoderFL = new CANCoder(1);
        encoderFL.configMagnetOffset(-195.381);
        encoderFL.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
        final SDSSwerveModule frontLeft = new SDSSwerveModule(new Translation2d(MODULE_OFFSET_XY, MODULE_OFFSET_XY),
                encoderFL, frontLeftSteer, new PIDSparkMax(1, MotorType.kBrushless), SDSSwerveModule.MK4_V2);

        // Front Right Module
        final CANCoder encoderFR = new CANCoder(2);
        encoderFR.configMagnetOffset(-304.189 + 180);
        encoderFR.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
        final SDSSwerveModule frontRight = new SDSSwerveModule(new Translation2d(MODULE_OFFSET_XY, -MODULE_OFFSET_XY),
                encoderFR, frontRightSteer, new PIDSparkMax(3, MotorType.kBrushless), SDSSwerveModule.MK4_V2);

        // Rear Left Module
        final CANCoder encoderRL = new CANCoder(3);
        encoderRL.configMagnetOffset(-298.213);
        encoderRL.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
        final SDSSwerveModule rearLeft = new SDSSwerveModule(new Translation2d(-MODULE_OFFSET_XY, MODULE_OFFSET_XY),
                encoderRL, rearLeftSteer, new PIDSparkMax(5, MotorType.kBrushless), SDSSwerveModule.MK4_V2);

        // Rear Right Module
        final CANCoder encoderRR = new CANCoder(4);
        encoderRR.configMagnetOffset(-168.223 + 180);
        encoderRR.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
        final SDSSwerveModule rearRight = new SDSSwerveModule(new Translation2d(-MODULE_OFFSET_XY, -MODULE_OFFSET_XY),
                encoderRR, rearRightSteer, new PIDSparkMax(7, MotorType.kBrushless), SDSSwerveModule.MK4_V2);

        final double maxDriveSpeedMetersPerSecond = Units.feetToMeters(12);

        final double maxRotationRadianPerSecond = Math.PI;

        return new SwerveDriveMap(frontLeft, frontRight, rearLeft, rearRight, maxDriveSpeedMetersPerSecond,
                maxRotationRadianPerSecond, pigeonGyro);

    }

    @Override
    public ShooterMap getShooterMap() {
        if (!shooterOn) {
            return super.getShooterMap();
        }
        final PIDSparkMax motor = new PIDSparkMax(16, MotorType.kBrushless);
        final PIDSparkMax follower = new PIDSparkMax(15, MotorType.kBrushless);

        final WEncoder encoder = new WEncoder(1, 2, true, EncodingType.k1X);

        final PIDController pid = new PIDController(0.82212 / 4.0, 0, 0);

        motor.getMotorController().setPeriodicFramePeriod(PeriodicFrame.kStatus3, 1000);
        follower.getMotorController().setPeriodicFramePeriod(PeriodicFrame.kStatus3, 1000);
        follower.getMotorController().setPeriodicFramePeriod(PeriodicFrame.kStatus0, 1000);

        SimpleMotorFeedforward ff = new SimpleMotorFeedforward(0.17243, 0.49992 / 4.0, 0.6289);

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

        if (!intakeOn) {
            return super.getIntakeMap();
        }
        // Current limit in amps
        final int CURRENT_LIMIT = 30;

        final PIDSparkMax deploymentMotor = new PIDSparkMax(11, MotorType.kBrushless);
        final PIDSparkMax deploymentFollower = new PIDSparkMax(12, MotorType.kBrushless);
        final PIDSparkMax rollerMotor = new PIDSparkMax(13, MotorType.kBrushless);
        final var rollerController = rollerMotor.getMotorController();

        rollerController.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 1000);
        deploymentMotor.getMotorController().setPeriodicFramePeriod(PeriodicFrame.kStatus3, 1000);
        deploymentFollower.getMotorController().setPeriodicFramePeriod(PeriodicFrame.kStatus3, 1000);
        deploymentFollower.getMotorController().setPeriodicFramePeriod(PeriodicFrame.kStatus0, 1000);
        for (PeriodicFrame pf : PeriodicFrame.values()) {
            rollerController.setPeriodicFramePeriod(pf, 500);
        }

        // Use current as a validator along with setting a current limit
        // on the motor controllers

        // deploymentMotor.validateCurrent(CURRENT_LIMIT);
        deploymentMotor.addValidator(
                new CurrentValidator(CURRENT_LIMIT, () -> deploymentMotor.getMotorController().getOutputCurrent(), 4));
        deploymentFollower.getMotorController().follow(deploymentMotor.getMotorController(), true);
        deploymentMotor.getMotorController().setSmartCurrentLimit(CURRENT_LIMIT);
        deploymentFollower.getMotorController().setSmartCurrentLimit(CURRENT_LIMIT);
        rollerMotor.getMotorController().setInverted(true);

        return new IntakeMap(deploymentMotor, rollerMotor);
    }

    @Override
    public BallTransportMap getBallTransportMap() {
        if (!ballTransportOn) {
            return super.getBallTransportMap();
        }
        final PIDSparkMax topMotor = new PIDSparkMax(14, MotorType.kBrushless);
        final PIDSparkMax bottomMotor = new PIDSparkMax(17, MotorType.kBrushless);

        topMotor.getMotorController().setIdleMode(IdleMode.kBrake);
        bottomMotor.getMotorController().setIdleMode(IdleMode.kBrake);
        final REVColorSensor colorSensor = new REVColorSensor(Port.kMXP);

        final WDigitalInput laserSwitch = new WDigitalInput(0);

        final var topController = topMotor.getMotorController();
        final var bottomController = bottomMotor.getMotorController();
        topController.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 1000);
        bottomController.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 1000);
        for (PeriodicFrame pf : PeriodicFrame.values()) {
            topController.setPeriodicFramePeriod(pf, 500);
            bottomController.setPeriodicFramePeriod(pf, 500);
        }
        return new BallTransportMap(bottomMotor, topMotor, colorSensor, laserSwitch);
    }

    @Override
    public ClimberMap getLeftClimberMap() {
        if (!climberOn) {
            return super.getLeftClimberMap();
        }
        // The current limit for the climber's motors in amps

        final PIDSparkMax extendMotor = new PIDSparkMax(9, MotorType.kBrushless);

        final PIDSparkMax rotateMotor = new PIDSparkMax(18, MotorType.kBrushless);

        final CANSparkMax extendController = extendMotor.getMotorController();
        final CANSparkMax rotateController = rotateMotor.getMotorController();

        extendController.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 1000);
        rotateController.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 1000);

        extendMotor.getMotorController().setIdleMode(IdleMode.kBrake);
        rotateMotor.getMotorController().setIdleMode(IdleMode.kBrake);
        extendMotor.getMotorController().setInverted(false);
        rotateMotor.getMotorController().setInverted(false);
        // Setting the current limits on both the validators and motor controllers
        // extendMotor.validateCurrent(CLIMBER_EXTEND_LIMIT);
        extendMotor.addValidator(new CurrentValidator(CLIMBER_EXTEND_LIMIT,
                () -> extendMotor.getMotorController().getOutputCurrent(), 4));

        extendMotor.getMotorController().setSmartCurrentLimit(CLIMBER_EXTEND_LIMIT);
        rotateMotor.addValidator(new CurrentValidator(CLIMBER_ROTATE_LIMIT, rotateController::getOutputCurrent, 4));
        rotateMotor.getMotorController().setSmartCurrentLimit(CLIMBER_ROTATE_LIMIT);

        return new ClimberMap(extendMotor, rotateMotor, extendController::getOutputCurrent,
                rotateController::getOutputCurrent, () -> pigeonGyro.getRaw().getPitch());
    }

    @Override
    public ClimberMap getRightClimberMap() {
        if (!climberOn) {
            return super.getRightClimberMap();
        }
        // The current limit for the climber's motors in amps

        final PIDSparkMax extendMotor = new PIDSparkMax(10, MotorType.kBrushless);

        final PIDSparkMax rotateMotor = new PIDSparkMax(19, MotorType.kBrushless);

        final CANSparkMax extendController = extendMotor.getMotorController();
        final CANSparkMax rotateController = rotateMotor.getMotorController();

        extendController.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 1000);
        rotateController.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 1000);

        extendMotor.getMotorController().setIdleMode(IdleMode.kBrake);
        rotateMotor.getMotorController().setIdleMode(IdleMode.kBrake);
        extendMotor.getMotorController().setInverted(true);
        rotateMotor.getMotorController().setInverted(true);

        // Setting the current limits on both the validators and motor controllers
        // extendMotor.validateCurrent(CLIMBER_EXTEND_LIMIT);
        extendMotor.addValidator(new CurrentValidator(CLIMBER_EXTEND_LIMIT,
                () -> extendMotor.getMotorController().getOutputCurrent(), 4));

        extendMotor.getMotorController().setSmartCurrentLimit(CLIMBER_EXTEND_LIMIT);
        rotateMotor.addValidator(new CurrentValidator(CLIMBER_ROTATE_LIMIT, rotateController::getOutputCurrent, 4));
        rotateMotor.getMotorController().setSmartCurrentLimit(CLIMBER_ROTATE_LIMIT);

        return new ClimberMap(extendMotor, rotateMotor, extendController::getOutputCurrent,
                rotateController::getOutputCurrent, () -> pigeonGyro.getRaw().getPitch());
    }

    @Override
    public LedMap getLedMap() {
        if (!ledOn) {
            return super.getLedMap();
        }
        AddressableLED led = new AddressableLED(0);
        // Best if this is a multiple of 10
        AddressableLEDBuffer ledBuffer = new AddressableLEDBuffer(30);

        SerialPort serialPort = new SerialPort(9600, SerialPort.Port.kMXP);

        return new LedMap(led, ledBuffer, serialPort);
    }
}
