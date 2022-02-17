package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.chopshop166.chopshoplib.commands.SmartSubsystemBase;
import com.chopshop166.chopshoplib.drive.SwerveModule;
import com.chopshop166.chopshoplib.motors.Modifier;

import org.ejml.dense.row.mult.SubmatrixOps_FDRM;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.maps.RobotMap.SwerveDriveMap;

public class Drive extends SmartSubsystemBase {

  private final SwerveDriveKinematics kinematics;
  private final SwerveModule frontLeft;
  private final SwerveModule frontRight;
  private final SwerveModule rearLeft;
  private final SwerveModule rearRight;

  private final double maxDriveSpeedMetersPerSecond;
  private final double maxRotationRadiansPerSecond;
  private final Gyro gyro;

  public Drive(final SwerveDriveMap map) {
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

  public CommandBase fieldCentricDrive(final DoubleSupplier translateX, final DoubleSupplier translateY,
      final DoubleSupplier rotation) {
    return running("Field Centric Drive", () -> updateSwerveSpeedAngle(translateX, translateY, rotation));
  }

  private void updateSwerveSpeedAngle(final DoubleSupplier translateX, final DoubleSupplier translateY,
      final DoubleSupplier rotation) {
    // Need to convert inputs from -1..1 scale to m/s
    final Modifier deadband = Modifier.deadband(0.12);
    final double translateXSpeed = deadband.applyAsDouble(translateX.getAsDouble()) * maxDriveSpeedMetersPerSecond;
    final double translateYSpeed = deadband.applyAsDouble(translateY.getAsDouble()) * maxDriveSpeedMetersPerSecond;
    final double rotationSpeed = deadband.applyAsDouble(rotation.getAsDouble()) * maxRotationRadiansPerSecond;
    SmartDashboard.putNumber("translate y", translateYSpeed);
    SmartDashboard.putNumber("translate x", translateXSpeed);
    SmartDashboard.putNumber("retation speed", rotationSpeed);
    final ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(translateYSpeed, translateXSpeed,
        rotationSpeed, Rotation2d.fromDegrees(-gyro.getAngle()));

    // Now use this in our kinematics
    final SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(speeds);

    // Front left module state
    frontLeft.setDesiredState(moduleStates[0]);

    // Front right module state
    frontRight.setDesiredState(moduleStates[1]);

    // Back left module state
    rearLeft.setDesiredState(moduleStates[2]);

    // Back right module state
    rearRight.setDesiredState(moduleStates[3]);
  }

  public CommandBase driveDistanceY(final double distance) {
    return functional("Drive Distance Y", () -> {
      frontLeft.resetDistance();
    }, () -> {
      updateSwerveSpeedAngle(() -> 0, () -> Math.signum(distance) * 0.2, () -> 0);
    }, (interrupted) -> {
      updateSwerveSpeedAngle(() -> 0, () -> 0, () -> 0);
    }, () -> {
      return Math.abs(frontLeft.getDistance()) >= Math.abs(distance);
    });
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void reset() {
    gyro.reset();

    // TODO Default wheels to start position?

  }

  @Override
  public void safeState() {
    final SwerveModuleState stop = new SwerveModuleState(0.0, new Rotation2d(0, 0));

    frontLeft.setDesiredState(stop);
    frontRight.setDesiredState(stop);
    rearLeft.setDesiredState(stop);
    rearRight.setDesiredState(stop);
  }
}
