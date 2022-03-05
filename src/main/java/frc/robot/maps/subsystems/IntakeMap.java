package frc.robot.maps.subsystems;

import java.util.function.DoubleSupplier;

import com.chopshop166.chopshoplib.motors.SmartMotorController;

public class IntakeMap {
    private final SmartMotorController deploymentMotor;
    private final SmartMotorController rollerMotor;

    private final DoubleSupplier current;
    private final DoubleSupplier current2;

    public IntakeMap() {
        this(new SmartMotorController(), new SmartMotorController(), () -> 0.0, () -> 0.0);
    }

    public IntakeMap(final SmartMotorController deploymentMotor, final SmartMotorController rollerMotor,
            final DoubleSupplier current, final DoubleSupplier current2) {

        this.rollerMotor = rollerMotor;

        this.deploymentMotor = deploymentMotor;

        this.current = current;

        this.current2 = current2;

    }

    public SmartMotorController getRoller() {
        return rollerMotor;
    }

    public SmartMotorController getDeploy() {
        return deploymentMotor;
    }

    public DoubleSupplier getCurrent() {
        return current;
    }

    public DoubleSupplier getCurrent2() {
        return current2;
    }

}