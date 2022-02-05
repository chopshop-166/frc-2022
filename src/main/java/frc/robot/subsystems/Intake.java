// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.BooleanSupplier;
import com.chopshop166.chopshoplib.commands.SmartSubsystemBase;
import com.chopshop166.chopshoplib.motors.Modifier;
import com.chopshop166.chopshoplib.motors.PIDControlType;
import com.chopshop166.chopshoplib.motors.SmartMotorController;
import com.chopshop166.chopshoplib.states.SpinDirection;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.maps.RobotMap.IntakeMap;

public class Intake extends SmartSubsystemBase {

    private final SmartMotorController deploymentMotor;
    private final SmartMotorController rollerMotor;

    private final BooleanSupplier insideLimit;
    private final BooleanSupplier outsideLimit;

    private static final double ROLLER_SPEED = 1;
    private static final double DEPLOY_SPEED = 1;

    private final Modifier insideModifier;
    private final Modifier outsideModifier;

    public Intake(final IntakeMap map) {
        this.deploymentMotor = map.getDeploy();
        this.rollerMotor = map.getRoller();
        this.insideLimit = map.getInsideLimit();
        this.outsideLimit = map.getOutsideLimit();
        this.outsideModifier = Modifier.upperLimit(outsideLimit);
        this.insideModifier = Modifier.lowerLimit(insideLimit);

        deploymentMotor.setControlType(PIDControlType.SmartMotion);
        deploymentMotor.setSetpoint(0.25);
    }

    // Counterclockwise ball go in, clockwise ball go out

    public CommandBase startIntakeMechanism(SpinDirection rollerDirection) {
        return startEnd("Start Intake Mechanism", () -> {
            rollerMotor.set(rollerDirection.get(ROLLER_SPEED));
            deploymentMotor.set(outsideModifier.applyAsDouble(DEPLOY_SPEED));
        }, () -> {
            rollerMotor.set(0);
            deploymentMotor.set(insideModifier.applyAsDouble(-DEPLOY_SPEED));
        });
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        SmartDashboard.putBoolean("Intake Deployed", !insideLimit.getAsBoolean());
        SmartDashboard.putNumber("Roller Speed", rollerMotor.get());
    }

    @Override
    public void safeState() {
        rollerMotor.stopMotor();
        deploymentMotor.stopMotor();

    }
}