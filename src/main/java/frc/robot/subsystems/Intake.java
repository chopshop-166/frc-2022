// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.BooleanSupplier;
import com.chopshop166.chopshoplib.commands.SmartSubsystemBase;
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

    public Intake(final IntakeMap map) {
        this.deploymentMotor = map.getDeploy();
        this.rollerMotor = map.getRoller();
        this.insideLimit = map.getInsideLimit();
        this.outsideLimit = map.getOutsideLimit();
    }

    // Counterclockwise go in, clockwise go out
    public CommandBase setRollerSpeed(SpinDirection s) {
        return startEnd("Activate Roller",
                () -> {
                    rollerMotor.set(s.get(rollerSpeed));
                }, () -> {
                    rollerMotor.set(0);
                });
    }

    public CommandBase retractIntake() {
        return cmd("Retract Intake").onExecute(() -> {
            if (!insideLimit.getAsBoolean()) {
                deploymentMotor.set(-deploySpeed);
            }
        }).finishedWhen(insideLimit).onEnd((interupted) -> {
            deploymentMotor.set(0);
        });
    }

    public CommandBase deployIntake() {
        return cmd("Deploy Intake").onExecute(() -> {
            if (!outsideLimit.getAsBoolean()) {
                deploymentMotor.set(deploySpeed);
            }
        }).finishedWhen(outsideLimit).onEnd((interupted) -> {
            deploymentMotor.set(0);
        });
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        SmartDashboard.putBoolean("Intake Deployed", !insideLimit.getAsBoolean());
    }

    @Override
    public void safeState() {

    }
}