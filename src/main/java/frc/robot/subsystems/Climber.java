// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.chopshop166.chopshoplib.commands.SmartSubsystemBase;
import com.chopshop166.chopshoplib.motors.SmartMotorController;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.maps.RobotMap.ClimberMap;

public class Climber extends SmartSubsystemBase {

  private class Arm {
    private final SmartMotorController motor;
    private final DigitalInput limitUpper;
    private final DigitalInput limitLower;

    public Arm(SmartMotorController motor, DigitalInput limitUpper, DigitalInput limitLower) {
      this.motor = motor;
      this.limitUpper = limitUpper;
      this.limitLower = limitLower;
    }

    public boolean getUpper() {
      return limitUpper.get();
    }

    public boolean getLower() {
      return limitLower.get();
    }

    public void set(double speed) {
      motor.set(speed);
    }
  }

  Arm leftArm;
  Arm rightArm;

  public Climber(ClimberMap map) {
    leftArm = new Arm(map.getLeftMotor(), map.getLeftLowerLimit(), map.getLeftUpperLimit());
    rightArm = new Arm(map.getRightMotor(), map.getRightLowerLimit(), map.getRightUpperLimit());

  }

  public CommandBase extend() {
    return running("Extend", () -> {
      if (leftArm.getUpper()) {
        leftArm.set(0.0);
      } else {
        leftArm.set(1.0);
      }

      if (rightArm.getUpper()) {
        rightArm.set(0.0);
      } else {
        leftArm.set(1.0);
      }

    });
  }

  public CommandBase retract() {
    return running("Retract", () -> {
      if (leftArm.getLower()) {
        leftArm.set(0.0);
      } else {
        leftArm.set(-1.0);
      }

      if (rightArm.getLower()) {
        rightArm.set(0.0);
      } else {
        leftArm.set(-1.0);
      }
    });
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void safeState() {
    // TODO Auto-generated method stub

  }
}
