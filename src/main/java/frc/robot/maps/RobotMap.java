package frc.robot.maps;

import com.chopshop166.chopshoplib.motors.SmartMotorController;

import edu.wpi.first.wpilibj.DigitalInput;

public class RobotMap {
    public static class ArmMap {
        private final SmartMotorController motor;
        private final DigitalInput upperLimit;
        private final DigitalInput lowerLimit;

        public ArmMap(SmartMotorController motor, DigitalInput upperLimit, DigitalInput lowerLimit) {
            this.motor = motor;
            this.upperLimit = upperLimit;
            this.lowerLimit = lowerLimit;
        }

        public void stop() {
            motor.set(0.0);
        }

        public void extend(double speed) {
            if (upperLimit.get()) {
                motor.set(0.0);
            } else {
                motor.set(speed);
            }
        }

        public void retract(double speed) {
            if (lowerLimit.get()) {
                motor.set(0.0);
            } else {
                motor.set(-speed);
            }
        }
    }

    public static class ArmGroup extends ArmMap {
        private ArmMap arms[];

        public ArmGroup(ArmMap... arms) {
            super(new SmartMotorController(), new DigitalInput(0), new DigitalInput(0)); // This gets rid of "implicit
                                                                                         // super constructor is
                                                                                         // undefined" error. I don't
                                                                                         // know of a
                                                                                         // better solution to this

            this.arms = arms;
        }

        public void stop() {
            for (ArmMap arm : arms) {
                arm.stop();
            }
        }

        public void extend(double speed) {
            for (ArmMap arm : arms) {
                arm.extend(speed);
            }
        }

        public void retract(double speed) {
            for (ArmMap arm : arms) {
                arm.retract(speed);
            }
        }
    }

    public static class ClimberMap {

        private final ArmGroup arms;

        public ClimberMap(SmartMotorController leftMotor, SmartMotorController rightMotor, DigitalInput leftUpperLimit,
                DigitalInput rightUpperLimit, DigitalInput leftLowerLimit, DigitalInput rightLowerLimit) {
            ArmMap leftArm = new ArmMap(leftMotor, leftUpperLimit, leftLowerLimit);
            ArmMap rightArm = new ArmMap(rightMotor, rightUpperLimit, rightLowerLimit);

            arms = new ArmGroup(leftArm, rightArm);
        }

        public ClimberMap() {
            this(new SmartMotorController(), new SmartMotorController(), new DigitalInput(0), new DigitalInput(0),
                    new DigitalInput(0), new DigitalInput(0));
        }

        public ArmMap getArms() {
            return arms;
        }
    }

    public static ClimberMap ClimberMap() {
        return new ClimberMap();
    }
}
