// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.chopshop166.chopshoplib.commands.SmartSubsystemBase;
import com.chopshop166.chopshoplib.motors.SmartMotorController;
import com.chopshop166.chopshoplib.sensors.MockDigitalInput;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.maps.RobotMap.IntakeMap;

public class Intake extends SmartSubsystemBase {

    private final SmartMotorController deploymentMotor;
    private final SmartMotorController rollerMotor;
    private final MockDigitalInput insideLimit;
    private final MockDigitalInput outsideLimit;

    public Intake(final IntakeMap map) {
        this.deploymentMotor = map.getDeploy();
        this.rollerMotor = map.getRoller();
        this.limitSwitch = map.getLimitSwitch();
    }

    public CommandBase activateRoller() {
        return instant("Activate Roller", () -> {
            rollerMotor.set(1);
        });
    }

    public CommandBase deactivateRoller() {
        return instant("Deactivate Roller", () -> {
            rollerMotor.set(0);
        });
    }

    public CommandBase deployIntake() {
        // not yet
        return instant("Deploy Intake", () -> {

        });
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        SmartDashboard.putBoolean("Intake Deployed", !limitSwitch.getAsBoolean());
    }

    @Override
    public void safeState() {
        // TODO Auto-generated method stub

    }
}