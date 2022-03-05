package frc.robot.maps.subsystems;

import com.chopshop166.chopshoplib.motors.SmartMotorController;

public class ClimberMap {

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
