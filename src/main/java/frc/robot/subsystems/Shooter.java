package frc.robot.subsystems;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.chopshop166.chopshoplib.PersistenceCheck;
import com.chopshop166.chopshoplib.commands.SmartSubsystemBase;
import com.chopshop166.chopshoplib.motors.SmartMotorController;
import com.chopshop166.chopshoplib.sensors.IEncoder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.maps.RobotMap.ShooterMap;

public class Shooter extends SmartSubsystemBase {

  private final SmartMotorController shooterMotor;
  private final IEncoder shootEncoder;

  private static final double MAX_RPM = 5300;
  private static final double RPM_BUFFER = 10;
  private final double VEL_MUL;
  private double shootSpeed;

  public enum HubSpeed {
    OFF(0.0), LOW(0.2), HIGH(0.5);

    private double speed;

    private HubSpeed(double speed) {
      this.speed = speed * Shooter.getMaxRpm();
    }

    public double getSpeed() {
      return this.speed;
    }
  }

  public Shooter(ShooterMap shooterMap) {
    shooterMotor = shooterMap.getShooterMotor();
    shootEncoder = shooterMotor.getEncoder();
    VEL_MUL = shooterMap.getVelocityMultiplier();
  }

  @Override
  public void periodic() {
    double velocity = shootEncoder.getRate() * VEL_MUL;
    SmartDashboard.putNumber("Speed (feet per second)", velocity);
  }

  @Override
  public void safeState() {
    shooterMotor.set(0.0);
  }

  public CommandBase setTargetHub(HubSpeed hub) {
    return instant("Set Default Speed", () -> {
      shooterMotor.setSetpoint(hub.getSpeed());
      shootSpeed = hub.getSpeed();
    });
  }

  // Speed doesn't need to be set here, since it is already set in
  // setTargetHub/setSpeed
  public CommandBase waitUntilSpeedUp() {
    BooleanSupplier check = () -> Math.abs(shootEncoder.getRate())
        - Math.min(MAX_RPM, shootSpeed) < RPM_BUFFER;
    PersistenceCheck p = new PersistenceCheck(5, check);
    return cmd("Wait Until Speed Up").finishedWhen(p);
  }

  public CommandBase setSpeed(DoubleSupplier speed) {
    return instant("Set Speed", () -> {
      double speeds = speed.getAsDouble();
      shooterMotor.setSetpoint(speeds * speeds * MAX_RPM);
    });
  }

  public static double getMaxRpm() {
    return MAX_RPM;
  }
}
