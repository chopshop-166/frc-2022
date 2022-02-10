// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.chopshop166.chopshoplib.commands.SmartSubsystemBase;
import com.chopshop166.chopshoplib.motors.SmartMotorController;
import com.chopshop166.chopshoplib.sensors.IEncoder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.maps.RobotMap.ShooterMap;

public class Shooter extends SmartSubsystemBase {
  private SmartMotorController shooterMotor;
  private SmartMotorController intakeMotor;

  private double defaultSpeedNums[] = { 0.0, 0.5, 1.0 };
  private double shootSpeed;
  private double shootPower;
  private double velocity;

  private double PIDconst = 0.1;
  private double RPMmul = 10000;

  private double shootWheelWidth = 2;
  private double waitTimeMul = 0.5;
  private double waitTime = 1.0;
  private double SHOOTTIME = 2;
  private double INTAKESPEED = .5; // ! must be between 0 and 1

  // ! constructiong motorControllers
  public Shooter(ShooterMap shooterMap) {
    // ** assign motorcontrollers from shootermap
    shooterMotor = shooterMap.getShooterMotor();
    intakeMotor = shooterMap.getIntakeMotor();
  }

  @Override
  public void periodic() {
    IEncoder shooterEncoder = shooterMotor.getEncoder();
    double error = (shootSpeed * RPMmul) - shooterEncoder.getRate(); // ? gets error for PID
    shootPower += error * PIDconst / RPMmul; // * we change the power based on error.
    waitTime = (error / RPMmul) * waitTimeMul; // * changes the time you have to wait based on how close
    // * you are to target speed;
    intakeMotor.set(shootPower); // * sets speed
    velocity = shootWheelWidth * Math.PI * shootSpeed * RPMmul / (60 * 12);
    // ? diamiter*pi(curcumfrence)* rmp (inches per min) / 60 (inches per second)
    // ? / 12 (feet per second)
    SmartDashboard.putNumber("Speed", velocity);
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

  public class shoot extends CommandBase {
    int b = 0;

    @Override
    public void initialize() {
      intakeMotor.set(INTAKESPEED); // ? i dont think that the intakespeed needs to be as accurate as
      // ? the shoot motor. If i am wrong about this, can always add a PID controller
      // ?to make it accurate.
    }

    @Override
    public void execute() {
      b++;
    }

    @Override
    public boolean isFinished() {
      return b >= SHOOTTIME * 50;
    }

    @Override
    public void end(boolean interrupted) {
      intakeMotor.set(0.0);
    }
  }

}
