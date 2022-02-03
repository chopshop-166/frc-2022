// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.BooleanSupplier;
import com.chopshop166.chopshoplib.commands.SmartSubsystemBase;
import com.chopshop166.chopshoplib.motors.Modifier;
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

    private final double rollerSpeed = 1;
    private final double deploySpeed = 1;

    private final Modifier insideModifier;
    private final Modifier outsideModifier;

    public Intake(final IntakeMap map) {
        this.deploymentMotor = map.getDeploy();
        this.rollerMotor = map.getRoller();
        this.insideLimit = map.getInsideLimit();
        this.outsideLimit = map.getOutsideLimit();
        outsideModifier = Modifier.upperLimit(outsideLimit);
        insideModifier = Modifier.lowerLimit(insideLimit);
        // :) we don't appreciate
        // deploymentMotor.setControlType(CANSparkMax.ControlType.kSmartMotion);
    }

    // Counterclockwise ball go in, clockwise ball go out
    public CommandBase setRollerSpeed(SpinDirection rDirection) {
        return startEnd("Activate Roller",
                () -> {
                    rollerMotor.set(rDirection.get(rollerSpeed));
                }, () -> {
                    rollerMotor.set(0);
                });
    }

    public CommandBase retractIntake() {
        return startEnd("Retract Intake", () -> {
            deploymentMotor.set(insideModifier.applyAsDouble(-deploySpeed));
        }, () -> {
            deploymentMotor.set(0);
        });
    }

    public CommandBase deployIntake() {
        return startEnd("Deploy Intake", () -> {
            deploymentMotor.set(outsideModifier.applyAsDouble(deploySpeed));
        }, () -> {
            deploymentMotor.set(0);
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
        rollerMotor.set(0);
        deploymentMotor.set(0);
    }
}