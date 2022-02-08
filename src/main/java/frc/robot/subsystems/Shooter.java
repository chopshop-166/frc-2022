// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.chopshop166.chopshoplib.commands.SmartSubsystemBase;
import com.chopshop166.chopshoplib.motors.SmartMotorController;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.maps.RobotMap.ShooterMap;

public class Shooter extends SmartSubsystemBase {
  // private SmartMotorController lateralMotor;
  // private SmartMotorController longitudinalMotor;
  private SmartMotorController shooterMotor;
  private SmartMotorController intakeMotor;

  private double shootSpeed;
  private double shootPower;

  private double speedBuffer = .01;
  private double PIDconst = .01;
  private double RPMmul = 10000;

  private double waitTime = 1.0;
  private double SHOOTTIME = 2;
  private double INTAKESPEED = .5; // ! must be between 0 and 1

  // ! constructiong motorControllers
  public Shooter(ShooterMap shooterMap) {
    // ** assign motorcontrollers from shootermap
    // lateralMotor = shooterMap.getLateralMotor();
    // longitudinalMotor = shooterMap.getLongitudinalMotor();
    shooterMotor = shooterMap.getShooterMotor();
    intakeMotor = shooterMap.getIntakeMotor();
  }

  @Override
  public void periodic() {
    double error = (shootSpeed * RPMmul) - shooterMotor.getEncoder().getRate(); // ? gets error for PID
    if (Math.abs(error) > speedBuffer) { // * if we are not close enought to the set speed then...
      shootPower += error * PIDconst / RPMmul; // * we changee the power based on error.
    }
    intakeMotor.set(shootPower);
  }

  @Override
  public void safeState() {
    // TODO Auto-generated method stub

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
