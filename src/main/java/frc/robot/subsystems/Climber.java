package frc.robot.subsystems;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.chopshop166.chopshoplib.commands.SmartSubsystemBase;
import com.chopshop166.chopshoplib.motors.Modifier;
import com.chopshop166.chopshoplib.motors.ModifierGroup;
import com.chopshop166.chopshoplib.motors.PIDSparkMax;
import com.chopshop166.chopshoplib.motors.SmartMotorController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.maps.RobotMap.TelescopeMap;

public class Climber extends SmartSubsystemBase {

  // Constants:
  private final double EXTEND_SPEED = 1.0;
  private final double RETRACT_SPEED = -1.0;
  private final double CURRENT_LIMIT = 1.0; // Current limit in amps, motor stops when this is reached

  private final SmartMotorController motor;
  private final BooleanSupplier upperLimit;
  private final BooleanSupplier lowerLimit;

  private final Modifier upperModifier;
  private final Modifier lowerModifier;
  private final ModifierGroup modifiers;

  public Climber(TelescopeMap map) {
    motor = map.getMotor();
    upperLimit = map.getUpperLimit();
    lowerLimit = map.getLowerLimit();
    upperModifier = Modifier.upperLimit(upperLimit);
    lowerModifier = Modifier.lowerLimit(lowerLimit);
    modifiers = new ModifierGroup(upperModifier, lowerModifier);
  }

  // Move the motor based on a variable speed and stop when motor current exceeds
  // CURRENT_LIMIT
  public CommandBase moveCurrent(DoubleSupplier speed) {
    return cmd("Move").onInitialize(() -> {
    }).onExecute(() -> {
      motor.set(speed.getAsDouble());
      SmartDashboard.putNumber("Climber Speed", speed.getAsDouble());
    }).finishedWhen(() -> ((PIDSparkMax) motor).getMotorController().getOutputCurrent() >= CURRENT_LIMIT)
        .onEnd((interrupted) -> {
          motor.set(0.0);
        });
  }

  // Move the motor based on a variable speed and stop when limit switches are hit
  public CommandBase move(DoubleSupplier speed) {
    return cmd("Move").onInitialize(() -> {
    }).onExecute(() -> {
      motor.set(modifiers.run(speed.getAsDouble()));
      SmartDashboard.putNumber("Climber Speed", speed.getAsDouble());
    }).onEnd((interrupted) -> {
      motor.set(0.0);
    });
  }

  // Extend the climber then stop the climber once the motor's current reaches a
  // certain threshold, specified by CURRENT_LIMIT
  public CommandBase extendCurrent() {
    return cmd("Extend With Current Limits").onInitialize(() -> {
      motor.set(EXTEND_SPEED);
    }).finishedWhen(() -> ((PIDSparkMax) motor).getMotorController().getOutputCurrent() >= CURRENT_LIMIT)
        .onEnd((interrupted) -> {
        });
  }

  public CommandBase retractCurrent() {
    return cmd("Retract With Current Limits").onInitialize(() -> {
      motor.set(RETRACT_SPEED);
    }).finishedWhen(() -> ((PIDSparkMax) motor).getMotorController().getOutputCurrent() >= CURRENT_LIMIT)
        .onEnd((interrupted) -> {
        });
  }

  // Extend the climber and use the limit switches to stop the motors
  public CommandBase extend() {
    return startEnd("Extend", () -> {
      motor.set(upperModifier.applyAsDouble(EXTEND_SPEED));
    }, () -> {
      motor.set(0.0);
    });
  }

  public CommandBase retract() {
    return startEnd("Retract", () -> {
      motor.set(lowerModifier.applyAsDouble(RETRACT_SPEED));
    }, () -> {
      motor.set(0.0);
    });
  }

  // Extend and ignore limits (only use if limits are not working)
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
