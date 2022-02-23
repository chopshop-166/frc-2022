package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.chopshop166.chopshoplib.commands.SmartSubsystemBase;
import com.chopshop166.chopshoplib.motors.Modifier;
import com.chopshop166.chopshoplib.motors.SmartMotorController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.maps.RobotMap.ClimberMap;

public class Climber extends SmartSubsystemBase {

  // Constants:
  private final double EXTEND_SPEED = 1.0;
  private final double RETRACT_SPEED = -1.0;

  private final SmartMotorController extendMotor;
  private final SmartMotorController rotateMotor;

  private final Modifier extendLimit;
  private final Modifier rotateLimit;

  public Climber(ClimberMap map) {

    extendMotor = map.getExtendMotor();
    rotateMotor = map.getRotateMotor();

    extendLimit = Modifier.unless(extendMotor::errored);
    rotateLimit = Modifier.unless(extendMotor::errored);
  }

  // Move the motor based off a variable speed
  public CommandBase move(DoubleSupplier speed) {
    double nSpeed = speed.getAsDouble();
    return cmd("Move").onExecute(() -> {
      extendMotor.set(extendLimit.applyAsDouble(nSpeed));
      SmartDashboard.putNumber("Climber Speed", nSpeed);
    }).finishedWhen(extendMotor::errored).onEnd((interrupted) -> {
      extendMotor.set(0.0);
    });
  }

  public CommandBase extend() {
    return cmd("Extend").onExecute(() -> {
      extendMotor.set(extendLimit.applyAsDouble(EXTEND_SPEED));
    }).finishedWhen(extendMotor::errored).onEnd((interrupted) -> {
      extendMotor.set(0.0);
    });
  }

  public CommandBase retract() {
    return cmd("Retract").onExecute(() -> {
      extendMotor.set(extendLimit.applyAsDouble(RETRACT_SPEED));
    }).finishedWhen(extendMotor::errored).onEnd((interrupted) -> {
      extendMotor.set(0.0);
    });
  }

  public CommandBase stop() {
    return instant("Stop", () -> {
      extendMotor.set(0.0);
      rotateMotor.set(0.0);
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
