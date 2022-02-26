package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.chopshop166.chopshoplib.commands.SmartSubsystemBase;
import com.chopshop166.chopshoplib.motors.Modifier;
import com.chopshop166.chopshoplib.motors.SmartMotorController;
import com.chopshop166.chopshoplib.states.SpinDirection;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.maps.RobotMap.ClimberMap;

public class Climber extends SmartSubsystemBase {

  // Constants:

  private final double ROTATE_SPEED = 0.2;

  private final SmartMotorController extendMotor;
  private final SmartMotorController rotateMotor;

  private final Modifier extendLimit;
  private final Modifier rotateLimit;

  public enum ExtendDirection {
    EXTEND(0.2), RETRACT(-0.2);

    private final double direction;

    private ExtendDirection(double direction) {
      this.direction = direction;
    }

    public double get() {
      return direction;
    }
  }

  public Climber(ClimberMap map) {

    extendMotor = map.getExtendMotor();
    rotateMotor = map.getRotateMotor();

    extendLimit = Modifier.unless(extendMotor::errored);
    rotateLimit = Modifier.unless(extendMotor::errored);
  }

  // Move the motor based off a variable speed
  public CommandBase extendSpeed(DoubleSupplier speed) {
    return cmd("Extend Speed").onExecute(() -> {
      double nSpeed = speed.getAsDouble();
      extendMotor.set(extendLimit.applyAsDouble(nSpeed));
      SmartDashboard.putNumber("Climber Speed", nSpeed);
    }).finishedWhen(extendMotor::errored).onEnd((interrupted) -> {
      extendMotor.set(0.0);
    });
  }

  public CommandBase rotateSpeed(DoubleSupplier speed) {
    return cmd("Rotate Speed").onExecute(() -> {
      double nSpeed = speed.getAsDouble();
      rotateMotor.set(rotateLimit.applyAsDouble(nSpeed));
      SmartDashboard.putNumber("Rotate Speed", nSpeed);
    }).finishedWhen(rotateMotor::errored).onEnd((interrupted) -> {
      rotateMotor.set(0.0);
    });
  }

  public CommandBase rotate(SpinDirection direction) {
    return cmd("Rotate").onExecute(() -> {
      rotateMotor.set(rotateLimit.applyAsDouble(direction.get(ROTATE_SPEED)));
    }).finishedWhen(rotateMotor::errored).onEnd((interrupted) -> {
      rotateMotor.set(0.0);
    });
  }

  public CommandBase extend(ExtendDirection direction) {
    return cmd("Extend").onExecute(() -> {
      extendMotor.set(extendLimit.applyAsDouble(direction.get()));
    }).finishedWhen(extendMotor::errored).onEnd((interrupted) -> {
      extendMotor.set(0.0);
    });
  }

  @Override
  public void safeState() {
    extendMotor.set(0.0);
    rotateMotor.set(0.0);
  }

  @Override
  public void periodic() {
  }
}
