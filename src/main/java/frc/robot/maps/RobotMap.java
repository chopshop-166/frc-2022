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
            this.deploymentMotor = new SmartMotorController();

            this.rollerMotor = new SmartMotorController();

            this.insideLimit = new MockDigitalInput();

            this.outsideLimit = new MockDigitalInput();
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

    public IntakeMap getIntakeMap() {
        return new IntakeMap(new SmartMotorController(), new SmartMotorController(), new MockDigitalInput(),
                new MockDigitalInput());
    }
}
