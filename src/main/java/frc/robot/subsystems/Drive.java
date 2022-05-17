package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.chopshop166.chopshoplib.commands.SmartSubsystemBase;
import com.chopshop166.chopshoplib.drive.SwerveDriveMap;
import com.chopshop166.chopshoplib.drive.SwerveModule;
import com.chopshop166.chopshoplib.motors.Modifier;
import com.chopshop166.chopshoplib.sensors.gyro.SmartGyro;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
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
    private final SmartGyro gyro;
    private boolean inverted;

    private double speedCoef = 1.0;

    private Field2d field = new Field2d();

    private double rotationOffset = 0.0;
    private double startingRotation = 0.0;

    SendableChooser<Double> startingAngleChooser = new SendableChooser<>();

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

        // These angles need some tweaking
        startingAngleChooser.addOption("Left Hub", -223.88);
        startingAngleChooser.addOption("Zero", 0.0);
        SmartDashboard.putData("Starting Angle", startingAngleChooser);
    }

    public CommandBase setRotationOffset() {
        return instant("Set Rotation Offset", () -> {
            rotationOffset = gyro.getRotation2d().getDegrees();
        });
    }

    // This sets an offset for the gyro when the robot is turned on. This offset can
    // be selected using the sendable chooser depending on where the robot is
    // positioned and facing in the beginning of the match

    public CommandBase resetRotationOffset() {
        return instant("Reset Rotation Offset", () -> {
            rotationOffset = 0.0;
        });
    }

    public CommandBase setSpeedCoef(double fac) {

        return instant("Set speedCoef", () -> {
            speedCoef = fac;
        });
    }

    public CommandBase fieldCentricDrive(final DoubleSupplier translateX, final DoubleSupplier translateY,
            final DoubleSupplier rotation) {
        return running("Field Centric Drive",
                () -> updateSwerveSpeedAngle(translateX, translateY, rotation));
    }

    private void updateSwerveSpeedAngle(final DoubleSupplier translateX, final DoubleSupplier translateY,
            final DoubleSupplier rotation) {
        // Need to convert inputs from -1..1 scale to m/s
        SmartDashboard.putNumber("Rotation Offset", rotationOffset);
        final Modifier deadband = Modifier.deadband(0.15);
        final double translateXSpeed = deadband.applyAsDouble(translateX.getAsDouble()) * maxDriveSpeedMetersPerSecond
                * speedCoef;
        final double translateYSpeed = deadband.applyAsDouble(translateY.getAsDouble()) * maxDriveSpeedMetersPerSecond
                * speedCoef;
        final double rotationSpeed = deadband.applyAsDouble(rotation.getAsDouble()) * maxRotationRadiansPerSecond;
        SmartDashboard.putNumber("Translate Y", translateYSpeed);
        SmartDashboard.putNumber("Translate X", translateXSpeed);
        SmartDashboard.putNumber("Rotation Speed", rotationSpeed);

        // rotationOffset is temporary and startingRotation is set at the start
        final ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(translateYSpeed, translateXSpeed,
                rotationSpeed, Rotation2d.fromDegrees(gyro.getAngle() - rotationOffset - startingRotation));

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
            startingRotation = 0.0;
        });
    }

    public void setInverted(boolean invertState) {
        inverted = invertState;

        frontLeft.setInverted(invertState);
        frontRight.setInverted(invertState);
        rearLeft.setInverted(invertState);
        rearRight.setInverted(invertState);
    }

    public void setModuleStates(SwerveModuleState[] states) {
        // Front left module state
        frontLeft.setDesiredState(states[0]);

        // Front right module state
        frontRight.setDesiredState(states[1]);

        // Back left module state
        rearLeft.setDesiredState(states[2]);

        // Back right module state
        rearRight.setDesiredState(states[3]);
    }

    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        gyro.setAngle(pose.getRotation().getDegrees() + 90);
        odometry.resetPosition(pose, gyro.getRotation2d());
        startingRotation = (pose.getRotation().getDegrees() - 90) % 180;

    }

    public CommandBase auto(PathPlannerTrajectory path, double thetaP) {
        ProfiledPIDController thetaController = new ProfiledPIDController(thetaP, 0, 0,
                new TrapezoidProfile.Constraints(Math.PI, Math.PI));
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        // Create a PPSwerveControllerCommand. This is almost identical to WPILib's
        // SwerveControllerCommand, but it uses the holonomic rotation
        // from the PathPlannerTrajectory to control the robot's rotation.
        // See the WPILib SwerveControllerCommand for more info on what you need to pass
        // to the command
        return new PPSwerveControllerCommand(
                path,
                this::getPose,
                kinematics,
                new PIDController(0.003, 0, 0),
                new PIDController(
                        0.007, 0, 0),
                thetaController,
                this::setModuleStates,
                this).andThen(instant("Stop", this::safeState));
    }

    public CommandBase autoInverted(PathPlannerTrajectory path, double thetaP) {
        return sequence("Invert Auto", instant("Set Invert True", () -> setInverted(true)), auto(path, thetaP),
                instant("Set Invert False", () -> setInverted(false)));
    }

    public CommandBase resetAuto(PathPlannerTrajectory initPath) {
        return instant("Reset Auto", () -> {
            PathPlannerState startingState = initPath.getInitialState();
            Pose2d startingPose = new Pose2d(startingState.poseMeters.getTranslation(),
                    startingState.holonomicRotation);
            resetOdometry(startingPose);
        });
    }

    @Override
    public void periodic() {
        odometry.update(Rotation2d.fromDegrees(gyro.getAngle() - 180), frontLeft.getState(), frontRight.getState(),
                rearLeft.getState(),
                rearRight.getState());

        field.setRobotPose(getPose());
        SmartDashboard.putNumber("Drive Gyro Angle", gyro.getAngle());
        SmartDashboard.putNumber("Drive Starting Angle", startingRotation);
        SmartDashboard.putData("Front Right", frontRight);
        SmartDashboard.putData("Front Left", frontLeft);
        SmartDashboard.putData("Rear Right", rearRight);
        SmartDashboard.putData("Rear Left", rearLeft);

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
                initialPose = new Pose2d(getPose().getTranslation().times(1), getPose().getRotation().times(1));
            }

            @Override
            public void execute() {
                updateSwerveSpeedAngle(() -> rotation.getSin() * speed, () -> rotation.getCos() * speed, () -> 0);
            }

            @Override
            public boolean isFinished() {
                return initialPose.getTranslation().getDistance(getPose().getTranslation()) >= distanceMeters;
            }

            @Override
            public void end(boolean interrupted) {
                updateSwerveSpeedAngle(() -> 0, () -> 0, () -> 0);
            }

        };
    }

    @Override
    public void reset() {
        gyro.reset();

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
