// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.chopshop166.chopshoplib.commands.SmartSubsystemBase;
import com.chopshop166.chopshoplib.motors.SmartMotorController;

import java.util.function.BooleanSupplier;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.maps.RobotMap.TelescopeMap;

public class Climber extends SmartSubsystemBase {

  // Constants:
  private final double EXTEND_SPEED = 1.0;
  private final double RETRACT_SPEED = 1.0;

  private final SmartMotorController motor;
  private final BooleanSupplier upperLimit;
  private final BooleanSupplier lowerLimit;

  public Climber(TelescopeMap map) {
    motor = map.getMotor();
    upperLimit = map.getUpperLimit();
    lowerLimit = map.getLowerLimit();
  }

  public CommandBase extend() {
    return cmd("Extend").onInitialize(() -> { // Extend and stop when limit is hit
      motor.set(EXTEND_SPEED);
    }).finishedWhen(upperLimit).onEnd((interrupted) -> {
      motor.set(0.0);
    });
  }

  public CommandBase retract() {
    return cmd("Retract").onInitialize(() -> {
      motor.set(-RETRACT_SPEED);
    }).finishedWhen(lowerLimit).onEnd((interrupted) -> { // Retract and stop when limit is hit
      motor.set(0.0);
    });
  }

  public CommandBase extendIgnoreLimit() {
    return startEnd("Extend Ignore Limit", () -> { // Extend and ignore limits (only use if limits are not working)
      motor.set(EXTEND_SPEED);
    }, () -> {
      motor.set(0.0);
    });
  }

  public CommandBase retractIgnoreLimit() {
    return startEnd("Retract Ignore Limit", () -> {
      motor.set(RETRACT_SPEED);
    }, () -> {
      motor.set(0.0);
    });
  }

  public CommandBase stop() {
    return instant("Stop", () -> {
      motor.set(0.0);
    });
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void safeState() {
    motor.set(0.0);
  }
}
