// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.chopshop166.chopshoplib.PersistenceCheck;
import com.chopshop166.chopshoplib.commands.SmartSubsystemBase;
import com.chopshop166.chopshoplib.motors.SmartMotorController;
import com.chopshop166.chopshoplib.sensors.IEncoder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.maps.RobotMap.ShooterMap;

public class Shooter extends SmartSubsystemBase {

  private final SmartMotorController shooterMotor;
  private final SmartMotorController loaderMotor;
  private final IEncoder shootEncoder;

  private final double VEL_MUL;
  private final double MAX_RPM = 5300;
  private final double WAIT_TIME = 1.0;
  private final double SHOOT_TIME = 2;
  private final double LOADING_SPEED = .5; // must be between 0 and 1
  private final double RPM_BUFFER = 10;
  private double shootSpeed;

  public enum HubSpeed {
    OFF(0.0), LOW(0.2), HIGH(0.5);

    private double speed;

    private HubSpeed(double speed) {
      this.speed = speed;
    }

    public double getSpeed() {
      return this.speed;
    }
  }

  public Shooter(ShooterMap shooterMap) {
    shooterMotor = shooterMap.getShooterMotor();
    loaderMotor = shooterMap.getLoadingMotor();
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
    loaderMotor.set(0.0);
  }

  public CommandBase setTargetHub(HubSpeed hub) {
    return instant("Set Default Speed", () -> {
      shooterMotor.setSetpoint(hub.getSpeed() * MAX_RPM);
      shootSpeed = hub.getSpeed() * MAX_RPM;
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
    double speeds = speed.getAsDouble();
    return running("Set Speed", () -> {
      shooterMotor.setSetpoint(speeds * speeds * MAX_RPM);
    });
  }

  public CommandBase shoot() {
    return sequence("Shoot",
        race("shoot race", new WaitCommand(WAIT_TIME), waitUntilSpeedUp()), fire());
  }

  public CommandBase fire() {
    return sequence("Shoot", setLoadingSpeed(LOADING_SPEED), new WaitCommand(SHOOT_TIME), setLoadingSpeed(0.0));
  }

  public CommandBase setLoadingSpeed(double speed) {
    return instant("Set Loading Speed", () -> {
      loaderMotor.set(speed);
    });
  }

}
