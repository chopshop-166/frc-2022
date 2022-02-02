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

  public Climber(TelescopeMap map) {
    motor = map.getMotor();
    upperLimit = map.getUpperLimit();
    lowerLimit = map.getLowerLimit();
  }

  public CommandBase move(DoubleSupplier speed) { // Move motor with variable speed that is affected by limit switches
    ModifierGroup limits = new ModifierGroup();
    limits.add(Modifier.upperLimit(upperLimit), Modifier.lowerLimit(lowerLimit));
    return cmd("Move").onInitialize(() -> {
      motor.set(limits.run(speed.getAsDouble()));
    }).onExecute(() -> {
      SmartDashboard.putNumber("Climber Speed", speed.getAsDouble());
    }).onEnd((interrupted) -> {
      motor.set(0.0);
    });
  }

  public CommandBase extend() {
    Modifier limit = Modifier.upperLimit(upperLimit);
    return startEnd("Extend", () -> {
      motor.set(limit.applyAsDouble(EXTEND_SPEED));
    }, () -> {
      motor.set(0.0);
    });
  }

  public CommandBase retract() {
    Modifier limit = Modifier.upperLimit(lowerLimit);
    return startEnd("Retract", () -> {
      motor.set(limit.applyAsDouble(RETRACT_SPEED));
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
