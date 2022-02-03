// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.chopshop166.chopshoplib.commands.SmartSubsystemBase;
import com.chopshop166.chopshoplib.motors.SmartMotorController;
import com.chopshop166.chopshoplib.sensors.IEncoder;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.maps.RobotMap.ShooterMap;

public class Shooter extends SmartSubsystemBase {
  // private SmartMotorController lateralMotor;
  // private SmartMotorController longitudinalMotor;
  private SmartMotorController shooterMotor;
  private SmartMotorController intakeMotor;
  private double shootSpeed;

  private double speedBuffer = .01;
  private double PIDconst = .01;
  private double RPMmul = 10000;

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
    shooterMotor.set(shootSpeed);
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

  public CommandBase setSpeed(DoubleSupplier speed) {
    return running("setSpeed", () -> {
      setSpeedF(speed.getAsDouble() * speed.getAsDouble());
    });
  }

  // ? checks if speed the speed we want is the speed we have, with some wiggle
  // ?room. and only
  // ? finishes when the actuall speed gets to wanted speed. no clue if the
  // ?encoder gives off RMP
  // ? or power level. i assmed RPM
  public class CheckShootSpeed extends CommandBase {
    double power;
    double speed;
    double error;
    IEncoder encoder = shooterMotor.getEncoder();
    // public CheckShootSpeed(double powerC) {
    // power = powerC; // *takes value from 0-1
    // }

    @Override
    public void execute() {
      power = this.speed;
      speed = power * RPMmul; // * sets tatget RPM
      error = speed - encoder.getRate(); // * the amount off we ar from target RPM
      if (error >= speed + speedBuffer || error <= speed - speedBuffer) {
        power -= error * PIDconst; // * if we are outside of reasonable speed, we change speed base error
      }
      setSpeedF(power); // * changes speed to speed calculated
    }

    @Override
    public boolean isFinished() {
      return Math.abs(error) <= speedBuffer; // * ends command when error is low enough;
    }
  }

  public class shoot extends CommandBase {
    int b = 0;

    @Override
    public void initialize() {
      intakeMotor.set(INTAKESPEED); // * i dont think that the intakespeed needs to be as accurate as
      // * the shoot motor. If i am wrong about this, can always add a PID controller
      // *to make it accurate.
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
