// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.WaitCommand;
import com.chopshop166.chopshoplib.commands.SmartSubsystemBase;
import com.chopshop166.chopshoplib.motors.SmartMotorController;
import com.chopshop166.chopshoplib.sensors.IEncoder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.maps.RobotMap.ShooterMap;

public class Shooter extends SmartSubsystemBase {

  private SmartMotorController shooterMotor;
  private SmartMotorController intakeMotor;
  private IEncoder shootEncoder = shooterMotor.getEncoder();

  private double defaultSpeedNums[] = { 0.0, 0.5, 1.0 };
  private double shootSpeed;

  private double shootWheelWidth = 2;
  private double waitTime = 1.0;
  private double SHOOTTIME = 2;
  private double INTAKESPEED = .5; // must be between 0 and 1

  // constructiong motorControllers
  public Shooter(ShooterMap shooterMap) {
    // assign motorcontrollers from shootermap
    shooterMotor = shooterMap.getShooterMotor();
    intakeMotor = shooterMap.getIntakeMotor();
  }

  @Override
  public void periodic() {
    double velocity = shootWheelWidth * Math.PI * shootEncoder.getRate() / (60 * 12);
    // diameter*pi(curcumfrence)* rmp (inches per min) / 60 (inches per second)
    // / 12 (feet per second)
    shooterMotor.setSetpoint(shootSpeed);
    SmartDashboard.putNumber("Speed (feet per second)", velocity);
  }

  @Override
  public void safeState() {
    shooterMotor.set(0.0);
    intakeMotor.set(0.0);
  }

  public double getSpeed() {
    return this.shootSpeed;
  }

  public void setSpeedF(double speedC) {
    this.shootSpeed = speedC;
  }

  public double getWaitTime() {
    return this.waitTime;
  }

  public CommandBase setDefaultSpeed(int index) {
    return instant("Set default speed", () -> {
      setSpeedF(defaultSpeedNums[index]);
    });
  }

  public CommandBase setSpeed(DoubleSupplier speed) {
    return running("setSpeed", () -> {
      setSpeedF(speed.getAsDouble() * speed.getAsDouble());
    });
  }

  public CommandBase shoot() {
    return sequence("Shoot", setIntakeSpeed(INTAKESPEED), new WaitCommand(SHOOTTIME), setIntakeSpeed(0.0));
  }

  public CommandBase setIntakeSpeed(double speed) {
    return instant("Set intake speed", () -> {
      intakeMotor.set(speed);
    });
  }
}
