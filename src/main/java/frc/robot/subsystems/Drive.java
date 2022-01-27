// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.chopshop166.chopshoplib.commands.SmartSubsystemBase;
import com.chopshop166.chopshoplib.drive.SwerveModule;

import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import frc.robot.maps.RobotMap.DriveMap;

public class Drive extends SmartSubsystemBase {

  private final SwerveDriveKinematics kinematics;
  private final SwerveModule frontLeft;
  private final SwerveModule frontRight;
  private final SwerveModule rearLeft;
  private final SwerveModule rearRight;

  private final double maxDriveSpeedMetersPerSecond;
  private final double maxRotationRadiansPerSecond;
  private final Gyro gyro;

  public Drive(final DriveMap map) {
    super();

    frontLeft = map.getFrontLeft();
    frontRight = map.getFrontRight();
    rearLeft = map.getRearLeft();
    rearRight = map.getRearRight();
    kinematics = new SwerveDriveKinematics(frontLeft.getLocation(), frontRight.getLocation(),
        rearLeft.getLocation(), rearRight.getLocation());
    gyro = map.getGyro();
    maxDriveSpeedMetersPerSecond = map.getMaxDriveSpeedMetersPerSecond();
    maxRotationRadiansPerSecond = map.getMaxRotationRadianPerSecond();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void reset() {
    // This method will be called once per scheduler run
  }

  @Override
  public void safeState() {
    // TODO Stop all movement

  }
}
