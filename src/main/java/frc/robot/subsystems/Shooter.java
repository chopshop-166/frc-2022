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

  private SmartMotorController shooterMotor;
  private SmartMotorController loaderMotor;
  private IEncoder shootEncoder;

  private double shootWheelWidth;
  private double velMul;
  private double MAXRPM = 5000;
  private double waitTime = 1.0;
  private double SHOOTTIME = 2;
  private double LOADINGSPEED = .5; // must be between 0 and 1
  private double rpmBuffer;
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
    shootWheelWidth = shooterMap.getWheelDiameter();
    shootEncoder = shooterMotor.getEncoder();
    System.out.print(DefaultSpeed.OFF);
    velMul = shootWheelWidth * Math.PI / (60 * 12);
  }

  @Override
  public void periodic() {
    double velocity = shootEncoder.getRate() * velMul;
    // diameter*pi(curcumfrence)* rmp (inches per min) / 60 (inches per second)
    // / 12 (feet per second)
    SmartDashboard.putNumber("Speed (feet per second)", velocity);
  }

  @Override
  public void safeState() {
    shooterMotor.set(0.0);
    loaderMotor.set(0.0);
  }

  public double getWaitTime() {
    return this.waitTime;
  }

  public CommandBase setDefaultSpeed(DefaultSpeed speed) {
    return instant("Set default speed", () -> {
      shooterMotor.setSetpoint(speed.getSpeed() * MAXRPM);
      shootSpeed = speed.getSpeed() * MAXRPM;
    });
  }

  public CommandBase waitTilSpeedUp() {
    BooleanSupplier somethingMoreCohearant = () -> Math.abs(shootEncoder.getRate())
        - Math.min(MAXRPM, shootSpeed) < rpmBuffer;
    PersistenceCheck p = new PersistenceCheck(5, somethingMoreCohearant);
    return cmd("waits til speed up").finishedWhen(p);
  }

  public CommandBase setSpeed(DoubleSupplier speed) {
    double speeds = speed.getAsDouble() * MAXRPM;
    return running("setSpeed", () -> {
      shooterMotor.setSetpoint(speeds * speeds);
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
