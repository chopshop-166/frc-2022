// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.chopshop166.chopshoplib.commands.SmartSubsystemBase;
import com.chopshop166.chopshoplib.motors.PIDSparkMax;
import com.chopshop166.chopshoplib.motors.SmartMotorController;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.maps.RobotMap.ShooterMap;

public class Shooter extends SmartSubsystemBase {
  private SmartMotorController lateralMotor;
  private SmartMotorController longitudinalMotor;
  private SmartMotorController shooterMotor;
  private SmartMotorController intakeMotor;

  // ! constructiong motorControllers
  public Shooter(ShooterMap shooterMap) {
    // ** assign motorcontrollers from shootermap
    lateralMotor = shooterMap.getLateralMotor();
    longitudinalMotor = shooterMap.getLongitudinalMotor();
    shooterMotor = shooterMap.getShooterMotor();
    intakeMotor = shooterMap.getIntakeMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void safeState() {
    // TODO Auto-generated method stub

  }
}
