package frc.robot.maps;

import java.util.function.BooleanSupplier;

import com.chopshop166.chopshoplib.drive.MockSwerveModule;
import com.chopshop166.chopshoplib.drive.SwerveModule;
import com.chopshop166.chopshoplib.maps.RobotMapFor;
import com.chopshop166.chopshoplib.motors.SmartMotorController;
import com.chopshop166.chopshoplib.sensors.MockDigitalInput;
import com.chopshop166.chopshoplib.sensors.MockGyro;

import edu.wpi.first.math.geometry.Translation2d;
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

    public static class IntakeMap {
        private final SmartMotorController deploymentMotor;
        private final SmartMotorController rollerMotor;
        private final BooleanSupplier insideLimit;
        private final BooleanSupplier outsideLimit;

        public IntakeMap() {
            this(new SmartMotorController(), new SmartMotorController(), new MockDigitalInput(),
                    new MockDigitalInput());
        }

        public IntakeMap(final SmartMotorController deploymentMotor, final SmartMotorController rollerMotor,
                final BooleanSupplier outsideLimit, final BooleanSupplier insideLimit) {

            this.rollerMotor = rollerMotor;

            this.deploymentMotor = deploymentMotor;

            this.outsideLimit = outsideLimit;

            this.insideLimit = insideLimit;
        }

        public SmartMotorController getRoller() {
            return rollerMotor;
        }

        public SmartMotorController getDeploy() {
            return deploymentMotor;
        }

        public BooleanSupplier getInsideLimit() {
            return insideLimit;
        }

        public BooleanSupplier getOutsideLimit() {
            return outsideLimit;
        }
    }

    public class ShooterMap {
        private SmartMotorController shootMotor;
        private double velocityMultiplier;

        public ShooterMap(SmartMotorController shootMotorController, double velocityMultiplier) {
            shootMotor = shootMotorController;
            this.velocityMultiplier = velocityMultiplier;
        }

        public ShooterMap() {
            this(new SmartMotorController(), 1.0);
        }

        public SmartMotorController getShooterMotor() {
            return this.shootMotor;
        }

        public double getVelocityMultiplier() {
            return velocityMultiplier;
        }

    }

    public static class ClimberMap {
        private final SmartMotorController motor;
        private final BooleanSupplier upperLimit;
        private final BooleanSupplier lowerLimit;

        public ClimberMap(SmartMotorController motor, BooleanSupplier upperLimit, BooleanSupplier lowerLimit) {
            this.motor = motor;
            this.upperLimit = upperLimit;
            this.lowerLimit = lowerLimit;

        }

        public ClimberMap() {
            this(new SmartMotorController(), new MockDigitalInput(), new MockDigitalInput());
        }

        public SmartMotorController getMotor() {
            return motor;
        }

        public BooleanSupplier getUpperLimit() {
            return upperLimit;
        }

        public BooleanSupplier getLowerLimit() {
            return lowerLimit;
        }

    }

    public IntakeMap getIntakeMap() {
        return new IntakeMap();
    }

    public ClimberMap getLeftClimberMap() {
        return new ClimberMap();
    }

    public ShooterMap getShooterMap() {
        return new ShooterMap();
    }

    public ClimberMap getRightClimberMap() {
        return new ClimberMap();
    }
}
