package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import javax.xml.stream.events.Comment;

import com.chopshop166.chopshoplib.commands.SmartSubsystemBase;
import com.chopshop166.chopshoplib.drive.SwerveDriveMap;
import com.chopshop166.chopshoplib.drive.SwerveModule;
import com.chopshop166.chopshoplib.motors.Modifier;
import com.google.common.math.DoubleMath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class Drive extends SmartSubsystemBase {

    private final SwerveDriveKinematics kinematics;
    private final SwerveModule frontLeft;
    private final SwerveModule frontRight;
    private final SwerveModule rearLeft;
    private final SwerveModule rearRight;
    private final SwerveDriveOdometry odometry;

    private final double maxDriveSpeedMetersPerSecond;
    private final double maxRotationRadiansPerSecond;
    private final Gyro gyro;

    private double speedCoef = 1.0;

    private Field2d field = new Field2d();

    private Pose2d pose = new Pose2d();

    public Drive(final SwerveDriveMap map) {
        super();
        SmartDashboard.putData("Field", field);

        frontLeft = map.getFrontLeft();
        frontRight = map.getFrontRight();
        rearLeft = map.getRearLeft();
        rearRight = map.getRearRight();
        kinematics = new SwerveDriveKinematics(frontLeft.getLocation(), frontRight.getLocation(),
                rearLeft.getLocation(), rearRight.getLocation());
        gyro = map.getGyro();
        maxDriveSpeedMetersPerSecond = map.getMaxDriveSpeedMetersPerSecond();
        maxRotationRadiansPerSecond = map.getMaxRotationRadianPerSecond();

        odometry = new SwerveDriveOdometry(kinematics, gyro.getRotation2d());
    }

    public CommandBase setSpeedCoef(double fac) {

        return instant("Set speedCoef", () -> {
            speedCoef = fac;
        });
    }

    public CommandBase fieldCentricDrive(final DoubleSupplier translateX, final DoubleSupplier translateY,
            final DoubleSupplier rotation) {
        return running("Field Centric Drive", () -> updateSwerveSpeedAngle(translateX, translateY, rotation));
    }

    public Pose2d getPose() {
        return pose;
    }

    private void updateSwerveSpeedAngle(final DoubleSupplier translateX, final DoubleSupplier translateY,
            final DoubleSupplier rotation) {
        // Need to convert inputs from -1..1 scale to m/s
        final Modifier deadband = Modifier.deadband(0.15);
        final double translateXSpeed = deadband.applyAsDouble(translateX.getAsDouble()) * maxDriveSpeedMetersPerSecond
                * speedCoef;
        final double translateYSpeed = deadband.applyAsDouble(translateY.getAsDouble()) * maxDriveSpeedMetersPerSecond
                * speedCoef;
        final double rotationSpeed = deadband.applyAsDouble(rotation.getAsDouble()) * maxRotationRadiansPerSecond;
        SmartDashboard.putNumber("Translate Y", translateYSpeed);
        SmartDashboard.putNumber("Translate X", translateXSpeed);
        SmartDashboard.putNumber("Rotation Speed", rotationSpeed);
        final ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(translateYSpeed, translateXSpeed,
                rotationSpeed, Rotation2d.fromDegrees(gyro.getAngle()));

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

    public CommandBase resetGyro() {
        return instant("Reset Gyro", () -> {
            gyro.reset();
        });
    }

    public CommandBase driveDistance(final double distanceMeters, final double direction, final double speed) {

        Rotation2d rotation = Rotation2d.fromDegrees(direction);
        Drive thisDrive = this;

        return new CommandBase() {
            {
                addRequirements(thisDrive);
                setName("Drive Distance");
            }

            private Pose2d initialPose;

            @Override
            public void initialize() {
                initialPose = new Pose2d(pose.getTranslation().times(1), pose.getRotation().times(1));
            }

            @Override
            public void execute() {
                updateSwerveSpeedAngle(() -> rotation.getSin() * speed, () -> rotation.getCos() * speed, () -> 0);
            }

            @Override
            public boolean isFinished() {
                return initialPose.getTranslation().getDistance(pose.getTranslation()) >= distanceMeters;
            }

            @Override
            public void end(boolean interrupted) {
                updateSwerveSpeedAngle(() -> 0, () -> 0, () -> 0);
            }

        };
    }

    @Override
    public void periodic() {
        pose = odometry.update(gyro.getRotation2d(), frontLeft.getState(), frontRight.getState(), rearLeft.getState(),
                rearRight.getState());

        field.setRobotPose(pose);
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
