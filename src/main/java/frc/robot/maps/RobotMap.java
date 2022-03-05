package frc.robot.maps;

import java.util.function.BooleanSupplier;

import com.chopshop166.chopshoplib.drive.MockSwerveModule;
import com.chopshop166.chopshoplib.drive.SwerveModule;
import com.chopshop166.chopshoplib.maps.RobotMapFor;
import com.chopshop166.chopshoplib.motors.SmartMotorController;
import com.chopshop166.chopshoplib.sensors.IColorSensor;
import com.chopshop166.chopshoplib.sensors.MockColorSensor;
import com.chopshop166.chopshoplib.sensors.MockDigitalInput;
import com.chopshop166.chopshoplib.sensors.MockGyro;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.interfaces.Gyro;

@RobotMapFor("Default")
public class RobotMap {
    public static class SwerveDriveMap {
        // All Distances are in Meters

        private final SwerveModule frontLeft;
        private final SwerveModule frontRight;
        private final SwerveModule rearLeft;
        private final SwerveModule rearRight;
        private final double maxDriveSpeedMetersPerSecond;
        private final double maxRotationRadianPerSecond;
        private final Gyro gyro;

        private final double distanceFromCenter = 0.381;

        public SwerveDriveMap() {

            this.frontLeft = new MockSwerveModule(new Translation2d(distanceFromCenter, distanceFromCenter));

            this.frontRight = new MockSwerveModule(new Translation2d(distanceFromCenter, -distanceFromCenter));

            this.rearLeft = new MockSwerveModule(new Translation2d(-distanceFromCenter, distanceFromCenter));

            this.rearRight = new MockSwerveModule(new Translation2d(-distanceFromCenter, -distanceFromCenter));

            this.maxDriveSpeedMetersPerSecond = 2;

            this.maxRotationRadianPerSecond = Math.PI;

            this.gyro = new MockGyro();
        }

        public SwerveDriveMap(final SwerveModule frontLeft, final SwerveModule frontRight, final SwerveModule rearLeft,
                final SwerveModule rearRight, final double maxDriveSpeedMetersPerSecond,
                final double maxRotationRadianPerSecond, final Gyro gyro) {

            this.frontLeft = frontLeft;

            this.frontRight = frontRight;

            this.rearLeft = rearLeft;

            this.rearRight = rearRight;

            this.maxDriveSpeedMetersPerSecond = maxDriveSpeedMetersPerSecond;

            this.maxRotationRadianPerSecond = maxRotationRadianPerSecond;

            this.gyro = gyro;
        }

        public SwerveModule getFrontLeft() {
            return frontLeft;
        }

        public SwerveModule getFrontRight() {
            return frontRight;
        }

        public SwerveModule getRearLeft() {
            return rearLeft;
        }

        public SwerveModule getRearRight() {
            return rearRight;
        }

        public double getMaxDriveSpeedMetersPerSecond() {
            return maxDriveSpeedMetersPerSecond;
        }

        public double getMaxRotationRadianPerSecond() {
            return maxRotationRadianPerSecond;
        }

        public Gyro getGyro() {
            return gyro;
        }
    }

    public SwerveDriveMap getSwerveDriveMap() {
        return new SwerveDriveMap();
    }

    public static class BallTransportMap {
        private final SmartMotorController bottomMotor;
        private final SmartMotorController topMotor;
        private final IColorSensor colorSensor;
        private final BooleanSupplier laserSwitch;

        public BallTransportMap() {
            this.bottomMotor = new SmartMotorController();

            this.topMotor = new SmartMotorController();

            this.colorSensor = new MockColorSensor();

            this.laserSwitch = new MockDigitalInput();
        }

        public BallTransportMap(final SmartMotorController bottomMotor, final SmartMotorController topMotor,
                final IColorSensor colorSensor, final BooleanSupplier laserSwitch) {

            this.bottomMotor = bottomMotor;

            this.topMotor = topMotor;

            this.colorSensor = colorSensor;

            this.laserSwitch = laserSwitch;
        }

        public SmartMotorController getBottomMotor() {
            return bottomMotor;
        }

        public SmartMotorController getTopMotor() {
            return topMotor;
        }

        public IColorSensor getColorSensor() {
            return colorSensor;
        }

        public BooleanSupplier getLaserSwitch() {
            return laserSwitch;
        }
    }

    public BallTransportMap getBallTransportMap() {
        return new BallTransportMap();
    }

    public static class IntakeMap {
        private final SmartMotorController deploymentMotor;
        private final SmartMotorController rollerMotor;

        public IntakeMap() {
            this(new SmartMotorController(), new SmartMotorController());
        }

        public IntakeMap(final SmartMotorController deploymentMotor, final SmartMotorController rollerMotor) {

            this.rollerMotor = rollerMotor;

            this.deploymentMotor = deploymentMotor;

        }

        public SmartMotorController getRoller() {
            return rollerMotor;
        }

        public SmartMotorController getDeploy() {
            return deploymentMotor;
        }

    }

    public IntakeMap getIntakeMap() {
        return new IntakeMap();
    }

    public static class ClimberMap {

        private final SmartMotorController extendMotor;
        private final SmartMotorController rotateMotor;

        public ClimberMap(SmartMotorController extendMotor, SmartMotorController rotateMotor) {
            this.extendMotor = extendMotor;
            this.rotateMotor = rotateMotor;
        }

        public ClimberMap() {
            this(new SmartMotorController(), new SmartMotorController());
        }

        public SmartMotorController getExtendMotor() {
            return extendMotor;
        }

        public SmartMotorController getRotateMotor() {
            return rotateMotor;
        }

    }

    public ClimberMap getLeftClimberMap() {
        return new ClimberMap();
    }

    public ClimberMap getRightClimberMap() {
        return new ClimberMap();
    }

    public static class LedMap {
        private final AddressableLED led;
        private final AddressableLEDBuffer ledBuffer;

        public LedMap(AddressableLED led, AddressableLEDBuffer ledBuffer) {
            this.led = led;
            this.ledBuffer = ledBuffer;
        }

        public LedMap() {
            this(new AddressableLED(0), new AddressableLEDBuffer(0));
        }

        public AddressableLED getLed() {
            return led;
        }

        public AddressableLEDBuffer getLedBuffer() {
            return ledBuffer;
        }

    }

    public LedMap getLedMap() {
        return new LedMap();
    }
}
