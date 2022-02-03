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
    modifiers = new ModifierGroup();
    modifiers.add(upperModifier, lowerModifier);
  }

  public CommandBase move(DoubleSupplier speed) { // Move motor with variable speed that is affected by limit switches
    return cmd("Move").onInitialize(() -> {
      motor.set(modifiers.run(speed.getAsDouble()));
    }).onExecute(() -> {
      SmartDashboard.putNumber("Climber Speed", speed.getAsDouble());
    }).onEnd((interrupted) -> {
      motor.set(0.0);
    });
  }

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
  public void safeState() {
    motor.set(0.0);
  }

  @Override
  public void periodic() {
  }
}
