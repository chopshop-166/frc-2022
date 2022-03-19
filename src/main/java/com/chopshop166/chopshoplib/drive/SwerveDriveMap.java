package com.chopshop166.chopshoplib.drive;

import java.util.function.DoubleSupplier;

import com.chopshop166.chopshoplib.sensors.gyro.MockGyro;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.interfaces.Gyro;

public class SwerveDriveMap {
    // All Distances are in Meters

    private final SwerveModule frontLeft;
    private final SwerveModule frontRight;
    private final SwerveModule rearLeft;
    private final SwerveModule rearRight;
    private final double maxDriveSpeedMetersPerSecond;
    private final double maxRotationRadianPerSecond;
    private final Gyro gyro;

    private final double distanceFromCenter = 0.381;

    public SwerveDriveMap() {

        this.frontLeft = new MockSwerveModule(new Translation2d(distanceFromCenter, distanceFromCenter));

        this.frontRight = new MockSwerveModule(new Translation2d(distanceFromCenter, -distanceFromCenter));

        this.rearLeft = new MockSwerveModule(new Translation2d(-distanceFromCenter, distanceFromCenter));

        this.rearRight = new MockSwerveModule(new Translation2d(-distanceFromCenter, -distanceFromCenter));

        this.maxDriveSpeedMetersPerSecond = 2;

        this.maxRotationRadianPerSecond = Math.PI;

        this.gyro = new MockGyro();
    }

    public SwerveDriveMap(final SwerveModule frontLeft, final SwerveModule frontRight, final SwerveModule rearLeft,
            final SwerveModule rearRight, final double maxDriveSpeedMetersPerSecond,
            final double maxRotationRadianPerSecond, final Gyro gyro) {

        this.frontLeft = frontLeft;

        this.frontRight = frontRight;

        this.rearLeft = rearLeft;

        this.rearRight = rearRight;

        this.maxDriveSpeedMetersPerSecond = maxDriveSpeedMetersPerSecond;

        this.maxRotationRadianPerSecond = maxRotationRadianPerSecond;

        this.gyro = gyro;
    }

    public SwerveModule getFrontLeft() {
        return frontLeft;
    }

    public SwerveModule getFrontRight() {
        return frontRight;
    }

    public SwerveModule getRearLeft() {
        return rearLeft;
    }

    public SwerveModule getRearRight() {
        return rearRight;
    }

    public double getMaxDriveSpeedMetersPerSecond() {
        return maxDriveSpeedMetersPerSecond;
    }

    public double getMaxRotationRadianPerSecond() {
        return maxRotationRadianPerSecond;
    }

    public Gyro getGyro() {
        return gyro;
    }
}