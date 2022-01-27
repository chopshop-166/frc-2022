package frc.robot.maps;

import com.chopshop166.chopshoplib.motors.SmartMotorController;

import edu.wpi.first.wpilibj.DigitalInput;

public class RobotMap {
    public static class ClimberMap {
        private final SmartMotorController leftMotor;
        private final SmartMotorController rightMotor;
        private final DigitalInput leftUpperLimit;
        private final DigitalInput rightUpperLimit;
        private final DigitalInput leftLowerLimit;
        private final DigitalInput rightLowerLimit;

        public ClimberMap(SmartMotorController leftMotor, SmartMotorController rightMotor, DigitalInput leftUpperLimit,
                DigitalInput rightUpperLimit, DigitalInput leftLowerLimit, DigitalInput rightLowerLimit) {
            this.leftMotor = leftMotor;
            this.rightMotor = rightMotor;
            this.leftUpperLimit = leftUpperLimit;
            this.rightUpperLimit = rightUpperLimit;
            this.leftLowerLimit = leftUpperLimit;
            this.rightLowerLimit = rightLowerLimit;
        }

        public ClimberMap() {
            this(new SmartMotorController(), new SmartMotorController(), new DigitalInput(0), new DigitalInput(0), new DigitalInput(0), new DigitalInput(0), );
        }

        public SmartMotorController getLeftMotor() {
            return leftMotor;
        }

        public SmartMotorController getRightMotor() {
            return rightMotor;
        }

        public DigitalInput getLeftUpperLimit() {
            return leftUpperLimit;
        }

        public DigitalInput getRightUpperLimit() {
            return rightUpperLimit;
        }

        public DigitalInput getLeftLowerLimit() {
            return leftLowerLimit;

        }

        public DigitalInput getRightLowerLimit() {
            return rightLowerLimit;
        }
    }

    public ClimberMap ClimberMap() {
        return new ClimberMap();
    }
}
