// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.chopshop166.chopshoplib.commands.SmartSubsystemBase;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.maps.RobotMap.ArmMap;
import frc.robot.maps.RobotMap.ClimberMap;

public class Climber extends SmartSubsystemBase {

  private final double SPEED = 1.0;

  private final ArmMap arm;

  public Climber(ClimberMap map) {
    arm = map.getArm();
  }

  public CommandBase extend() {
    return running("Extend", () -> {
      arm.extend(SPEED);
    });
  }

  public CommandBase retract() {
    return running("Retract", () -> {
      arm.retract(SPEED);
    });
  }

  public CommandBase stop() {
    return instant("Stop", () -> {
      arm.stop();
    });
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void safeState() {
    arm.stop();

  }
}
