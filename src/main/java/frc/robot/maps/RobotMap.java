package frc.robot.maps;

import com.chopshop166.chopshoplib.maps.RobotMapFor;
import com.chopshop166.chopshoplib.motors.SmartMotorController;
import com.chopshop166.chopshoplib.sensors.MockDigitalInput;

import edu.wpi.first.wpilibj.DigitalInput;

// Need to get MAC address for roborio
@RobotMapFor("Default")
public class RobotMap {
    public static class IntakeMap {
        private final SmartMotorController deploymentMotor;
        private final SmartMotorController rollerMotor;
        private final DigitalInput insideLimit;
        private final DigitalInput outsideLimit;

        public IntakeMap() {
            this(new SmartMotorController(), new SmartMotorController(), new MockDigitalInput(),
                    new MockDigitalInput());
            this.deploymentMotor = new SmartMotorController();
        }

        public IntakeMap(final SmartMotorController deploymentMotor, final SmartMotorController rollerMotor,
                final DigitalInput outsideLimit, final DigitalInput insideLimit) {
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

        public DigitalInput getInsideLimit() {
            return insideLimit;
        }

        public DigitalInput getOutsideLimit() {
            return outsideLimit;
        }
    }

    public IntakeMap getIntakeMap() {
        return new IntakeMap(new SmartMotorController(), new SmartMotorController(), new MockDigitalInput(),
                new MockDigitalInput());
    }
}
