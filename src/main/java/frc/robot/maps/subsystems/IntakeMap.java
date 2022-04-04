package frc.robot.maps.subsystems;

import com.chopshop166.chopshoplib.motors.SmartMotorController;

public class IntakeMap {
    private final SmartMotorController deploymentMotor;
    private final SmartMotorController rollerMotor;

    public IntakeMap() {
        this(new SmartMotorController(), new SmartMotorController());
    }

    public IntakeMap(final SmartMotorController deploymentMotor, final SmartMotorController rollerMotor) {

        this.rollerMotor = rollerMotor;

        this.deploymentMotor = deploymentMotor;

    }

    public SmartMotorController getRoller() {
        return rollerMotor;
    }

    public SmartMotorController getDeploy() {
        return deploymentMotor;
    }

}