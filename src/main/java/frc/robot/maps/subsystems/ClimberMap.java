package frc.robot.maps.subsystems;

import java.util.function.DoubleSupplier;

import com.chopshop166.chopshoplib.motors.SmartMotorController;

public class ClimberMap {

    private final SmartMotorController extendMotor;
    private final SmartMotorController rotateMotor;
    private final DoubleSupplier extendCurrent;
    private final DoubleSupplier rotateCurrent;
    private final DoubleSupplier gyroPitch;

    public ClimberMap(SmartMotorController extendMotor, SmartMotorController rotateMotor, DoubleSupplier extendCurrent,
            DoubleSupplier rotateCurrent, DoubleSupplier gyroPitch) {
        this.extendMotor = extendMotor;
        this.rotateMotor = rotateMotor;
        this.extendCurrent = extendCurrent;
        this.rotateCurrent = rotateCurrent;
        this.gyroPitch = gyroPitch;

    }

    public ClimberMap() {
        this(new SmartMotorController(), new SmartMotorController(), () -> 0.0, () -> 0.0, () -> 0.0);
    }

    public SmartMotorController getExtendMotor() {
        return extendMotor;
    }

    public SmartMotorController getRotateMotor() {
        return rotateMotor;
    }

    public DoubleSupplier getExtendCurrent() {
        return extendCurrent;
    }

    public DoubleSupplier getRotateCurrent() {
        return rotateCurrent;
    }

    public DoubleSupplier getGyroPitch() {
        return gyroPitch;
    }
}
