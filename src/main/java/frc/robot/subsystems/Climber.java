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

  private final SmartMotorController motor;

  private final Modifier limit;

  public Climber(TelescopeMap map) {
    motor = map.getMotor();

    limit = Modifier.unless(motor::errored);
  }

  // Move the motor based on a variable speed and stop when limit switches are hit
  public CommandBase move(DoubleSupplier speed) {
    return cmd("Move").onExecute(() -> {
      motor.set(limit.applyAsDouble(speed.getAsDouble()));
      SmartDashboard.putNumber("Climber Speed", speed.getAsDouble());
    }).finishedWhen(motor::errored).onEnd((interrupted) -> {
      motor.set(0.0);
    });
  }

  // Extend the climber and use validators stop the motors
  public CommandBase extend() {
    return cmd("Extend").onExecute(() -> {
      motor.set(limit.applyAsDouble(EXTEND_SPEED));
    }).finishedWhen(motor::errored).onEnd((interrupted) -> {
      motor.set(0.0);
    });
  }

  public CommandBase retract() {
    return cmd("Extend").onExecute(() -> {
      motor.set(limit.applyAsDouble(RETRACT_SPEED));
    }).finishedWhen(motor::errored).onEnd((interrupted) -> {
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
