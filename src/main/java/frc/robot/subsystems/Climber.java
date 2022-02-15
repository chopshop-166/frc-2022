package frc.robot.subsystems;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.chopshop166.chopshoplib.commands.SmartSubsystemBase;
import com.chopshop166.chopshoplib.motors.Modifier;
import com.chopshop166.chopshoplib.motors.ModifierGroup;
import com.chopshop166.chopshoplib.motors.SmartMotorController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.maps.RobotMap.TelescopeMap;

public class Climber extends SmartSubsystemBase {

  // Constants:
  private final double EXTEND_SPEED = 1.0;
  private final double RETRACT_SPEED = -1.0;

  private final SmartMotorController motor;

  private final Modifier validatorLimit;
  private final ModifierGroup switchLimit;
  private final ModifierGroup limit;

  public Climber(TelescopeMap map) {
    Modifier upperLimit = Modifier.upperLimit(map.getUpperLimit());
    Modifier lowerLimit = Modifier.lowerLimit(map.getLowerLimit());
    motor = map.getMotor();
    switchLimit = new ModifierGroup(upperLimit , lowerLimit);
    validatorLimit = Modifier.unless(motor::errored);
    limit = new ModifierGroup(upperLimit, lowerLimit, validatorLimit)
  }

  // Move the motor based off a variable speed
  public CommandBase move(DoubleSupplier speed) {
    return cmd("Move").onExecute(() -> {
      motor.set(limit.run(speed.getAsDouble()));
    }).onEnd((interrupted) -> {
      motor.set(0.0);
    });
  }

  // Move the motor based on a variable speed and stop when validator fails
  public CommandBase moveCurrent(DoubleSupplier speed) {
    return cmd("Move With Current Limit").onExecute(() -> {
      motor.set(validatorLimit.applyAsDouble(speed.getAsDouble()));
      SmartDashboard.putNumber("Climber Speed", speed.getAsDouble());
    }).finishedWhen(motor::errored).onEnd((interrupted) -> {
      motor.set(0.0);
    });
  }

  // Move the motor with variable speed and stop when limit switches are hit
  public CommandBase moveLimit(DoubleSupplier speed) {
    return cmd("Move With Limits").onExecute(() -> {
      motor.set(switchLimit.run(speed.getAsDouble()));
    }).onEnd((interrupted) -> {
      motor.set(0.0);
    });
  }

  public CommandBase extend() {
    return cmd("Extend").onExecute(() -> {
      motor.set(limit.run(EXTEND_SPEED));
    }).onEnd((interrupted) -> {
      motor.set(0.0);
    });
  }

  // Extend the climber and use validators stop the motors
  public CommandBase extendCurrent() {
    return cmd("Extend With Current Limit").onExecute(() -> {
      motor.set(validatorLimit.applyAsDouble(EXTEND_SPEED));
    }).finishedWhen(motor::errored).onEnd((interrupted) -> {
      motor.set(0.0);
    });
  }

  // Extend the motor with limit switches
  public CommandBase extendLimit() {
    return cmd("Extend With Limits").onExecute(() -> {
      motor.set(switchLimit.run(EXTEND_SPEED));
    }).onEnd((interrupted) -> {
      motor.set(0.0);
    });
  }

  public CommandBase retract() {
    return cmd("Retract").onExecute(() -> {
      motor.set(limit.run(RETRACT_SPEED));
    }).onEnd((interrupted) -> {
      motor.set(0.0);
    });
  }

  public CommandBase retractCurrent() {
    return cmd("Retract With Current Limit").onExecute(() -> {
      motor.set(validatorLimit.applyAsDouble(RETRACT_SPEED));
    }).finishedWhen(motor::errored).onEnd((interrupted) -> {
      motor.set(0.0);
    });
  }

  // Retract the motor with limit switches
  public CommandBase retractLimit() {
    return cmd("Retract With Limits").onExecute(() -> {
      motor.set(switchLimit.run(RETRACT_SPEED));
    }).onEnd((interrupted) -> {
      motor.set(0.0);
    });
  }

  // Extend and ignore validation (only use if limits are not working)
  public CommandBase extendIgnoreLimit() {
    return startEnd("Extend Ignore Limit", () -> {
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
  public void safeState() {
    motor.set(0.0);
  }

  @Override
  public void periodic() {
  }
}
