package frc.robot.subsystems;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.chopshop166.chopshoplib.PersistenceCheck;
import com.chopshop166.chopshoplib.commands.SmartSubsystemBase;
import com.chopshop166.chopshoplib.motors.SmartMotorController;
import com.chopshop166.chopshoplib.sensors.IEncoder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.maps.RobotMap.ShooterMap;

public class Shooter extends SmartSubsystemBase {

  private final SmartMotorController motor;
  private final IEncoder encoder;

  private static final double MAX_RPM = 5300;
  private static final double RPM_BUFFER = 10;
  private double shootSpeed;

  public enum HubSpeed {
    LOW(1000), HIGH(2650);

    private double speed;

    private HubSpeed(double speed) {
      this.speed = speed;
    }

    public double get() {
      return this.speed;
    }
  }

  public Shooter(ShooterMap map) {
    motor = map.getMotor();
    encoder = map.getEncoder();
    shootSpeed = HubSpeed.LOW.get();
  }

  public Command testSpeed(double speed) {
    return startEnd("Test Speed", () -> {
      motor.set(speed);
    }, () -> {
      motor.set(0.0);
    });
  }

  public CommandBase setTargetHub(HubSpeed hub) {
    return instant("Set Default Speed", () -> {
      shootSpeed = hub.get();
      motor.setSetpoint(shootSpeed);
    });
  }

  public CommandBase stop() {
    return instant("Stop Shooter", this::safeState);
  }

  // Speed doesn't need to be set here, since it is already set in
  // setTargetHub/setSpeed
  public CommandBase waitUntilSpeedUp() {
    BooleanSupplier check = () -> Math.abs(encoder.getRate())
        - Math.min(MAX_RPM, shootSpeed) < RPM_BUFFER;
    PersistenceCheck p = new PersistenceCheck(5, check);
    return cmd("Wait Until Speed Up").until(p);
  }

  public CommandBase setSpeed(DoubleSupplier speed) {
    return instant("Set Speed", () -> {
      double speeds = speed.getAsDouble();
      motor.setSetpoint(speeds * speeds * MAX_RPM);
    });
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Error Amount", shootSpeed - encoder.getRate());
    SmartDashboard.putNumber("Encoder Rate", encoder.getRate());
  }

  @Override
  public void safeState() {
    motor.stopMotor();
  }
}
