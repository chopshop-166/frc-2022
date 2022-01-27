package frc.robot.maps;

import com.chopshop166.chopshoplib.maps.RobotMapFor;
import com.chopshop166.chopshoplib.motors.SmartMotorController;

// Need to get MAC address for roborio
@RobotMapFor("Default")
public class RobotMap {
    public class ShooterMap {
        private SmartMotorController lateralMotor;
        private SmartMotorController longitudinalMotor;
        private SmartMotorController shootMotor;
        private SmartMotorController intakeMotor;

        public ShooterMap(SmartMotorController lateralMotorController, SmartMotorController longitudinalMotorController,
                SmartMotorController shootMotorController, SmartMotorController intakeMotorController) {
            lateralMotor = lateralMotorController;
            longitudinalMotor = longitudinalMotorController;
            shootMotor = shootMotorController;
            intakeMotor = intakeMotorController;
        }

        public ShooterMap() {
            this(new SmartMotorController(), new SmartMotorController(), new SmartMotorController(),
                    new SmartMotorController());
        }

        public SmartMotorController getLateralMotor() {
            return this.lateralMotor;
        }

        public SmartMotorController getLongitudinalMotor() {
            return this.longitudinalMotor;
        }

        public SmartMotorController getShooterMotor() {
            return this.shootMotor;
        }

        public SmartMotorController getIntakeMotor() {
            return this.intakeMotor;
        }
    }

    public ShooterMap getShooterMap() {
        return new ShooterMap(new SmartMotorController(), new SmartMotorController(), new SmartMotorController(),
                new SmartMotorController());
    }
}
