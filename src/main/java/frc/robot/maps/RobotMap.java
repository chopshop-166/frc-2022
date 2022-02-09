package frc.robot.maps;

import java.util.function.BooleanSupplier;

import com.chopshop166.chopshoplib.maps.RobotMapFor;
import com.chopshop166.chopshoplib.motors.SmartMotorController;
import com.chopshop166.chopshoplib.sensors.MockDigitalInput;

@RobotMapFor("Default")
public class RobotMap {

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
        private SmartMotorController intakeMotor;

        public ShooterMap(SmartMotorController shootMotorController, SmartMotorController intakeMotorController) {
            shootMotor = shootMotorController;
            intakeMotor = intakeMotorController;
        }

        public ShooterMap() {
            this(new SmartMotorController(), new SmartMotorController());
        }

        public SmartMotorController getShooterMotor() {
            return this.shootMotor;
        }

        public SmartMotorController getIntakeMotor() {
            return this.intakeMotor;
        }
    }

    public static class TelescopeMap {
        private final SmartMotorController motor;
        private final BooleanSupplier upperLimit;
        private final BooleanSupplier lowerLimit;

        public TelescopeMap(SmartMotorController motor, BooleanSupplier upperLimit, BooleanSupplier lowerLimit) {
            this.motor = motor;
            this.upperLimit = upperLimit;
            this.lowerLimit = lowerLimit;
        }

        public TelescopeMap() {
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

    public TelescopeMap getLeftTelescopeMap() {
        return new TelescopeMap();
    }
    public ShooterMap getShooterMap(){
        return new ShooterMap();
    }
    public TelescopeMap getRightTelescopeMap() {
        return new TelescopeMap();
    }
}
