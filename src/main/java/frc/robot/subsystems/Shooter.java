// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.WaitCommand;

import com.chopshop166.chopshoplib.PersistenceCheck;
import com.chopshop166.chopshoplib.commands.SmartSubsystemBase;
import com.chopshop166.chopshoplib.motors.SmartMotorController;
import com.chopshop166.chopshoplib.sensors.IEncoder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.maps.RobotMap.ShooterMap;

public class Shooter extends SmartSubsystemBase {

  private final SmartMotorController shooterMotor;
  private final SmartMotorController loaderMotor;
  private final IEncoder shootEncoder;

  private final double SHOOT_WHEEL_WIDTH;
  private final double VEL_MUL;
  private final double MAX_RPM = 5000;
  private final double WAIT_TIME = 1.0;
  private final double SHOOTTIME = 2;
  private final double LOADINGSPEED = .5; // must be between 0 and 1
  private final double RPM_BUFFER = 10;
  private double shootSpeed;

  public enum DefaultSpeed {
    OFF(0.0), LOW(0.2), HIGH(0.5);

    private double speed;

    private DefaultSpeed(double speed) {
      this.speed = speed;
    }

    public double getSpeed() {
      return this.speed;
    }
  }

  // constructiong motorControllers
  public Shooter(ShooterMap shooterMap) {
    // assign motorcontrollers from shootermap
    shooterMotor = shooterMap.getShooterMotor();
    loaderMotor = shooterMap.getLoadingMotor();
    SHOOT_WHEEL_WIDTH = shooterMap.getWheelDiameter();
    shootEncoder = shooterMotor.getEncoder();
    VEL_MUL = SHOOT_WHEEL_WIDTH * Math.PI / (60 * 12);
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

  public double getWaitTime() {
    return this.WAIT_TIME;
  }

  public CommandBase setDefaultSpeed(DefaultSpeed speed) {
    return instant("Set default speed", () -> {
      shooterMotor.setSetpoint(speed.getSpeed() * MAX_RPM);
      shootSpeed = speed.getSpeed() * MAX_RPM;
    });
  }

  public CommandBase waitTilSpeedUp() {
    BooleanSupplier check = () -> Math.abs(shootEncoder.getRate())
        - Math.min(MAX_RPM, shootSpeed) < RPM_BUFFER;
    PersistenceCheck p = new PersistenceCheck(5, check);
    return cmd("waits til speed up").finishedWhen(p);
  }

  public CommandBase setSpeed(DoubleSupplier speed) {
    double speeds = speed.getAsDouble();
    return running("setSpeed", () -> {
      shooterMotor.setSetpoint(speeds * speeds * MAX_RPM);
    });
  }

  public CommandBase shoot() {
    return sequence("Shoot", setLoadingSpeed(LOADINGSPEED), new WaitCommand(SHOOTTIME), setLoadingSpeed(0.0));
  }

  public CommandBase setLoadingSpeed(double speed) {
    return instant("Set loading speed", () -> {
      loaderMotor.set(speed);
    });
  }

}
