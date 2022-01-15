package com.chopshop166.chopshoplib.drive;

import com.chopshop166.chopshoplib.motors.PIDControlType;
import com.chopshop166.chopshoplib.motors.PIDSparkMax;
import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.SendableBuilder;

/** Swerve Drive Specialties Mk3. */
public class SDSSwerveModule implements SwerveModule {

    public static class Configuration {

        /** Overall gear ratio for the swerve module drive motor. */
        public final double gearRatio;
        /** Wheel diameter. */
        public final double wheelDiameter;

        public Configuration(double gearRatio, double wheelDiameter) {
            this.gearRatio = gearRatio;
            this.wheelDiameter = wheelDiameter;
        }

        public double getConversion() {
            return gearRatio * Math.PI * wheelDiameter;
        }
    }

    public static final Configuration MK3_STANDARD = new Configuration(
            (14.0 / 50.0) * (28.0 / 16.0) * (15.0 / 60.0), Units.inchesToMeters(4));

    public static final Configuration MK3_FAST = new Configuration(
            (16.0 / 48.0) * (28.0 / 16.0) * (15.0 / 60.0), Units.inchesToMeters(4));

    public static final Configuration MK4_V1 = new Configuration(
            (14.0 / 50.0) * (25.0 / 19.0) * (15.0 / 45.0), Units.inchesToMeters(3.95));

    public static final Configuration MK4_V2 = new Configuration(
            (14.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0), Units.inchesToMeters(3.95));

    public static final Configuration MK4_V3 = new Configuration(
            (14.0 / 50.0) * (28.0 / 16.0) * (15.0 / 45.0), Units.inchesToMeters(3.95));

    public static final Configuration MK4_V4 = new Configuration(
            (16.0 / 48.0) * (28.0 / 16.0) * (15.0 / 45.0), Units.inchesToMeters(3.95));

    /** The physical location of the module. */
    private final Translation2d location;
    /** The encoder used for steering. */
    private final CANCoder steeringEncoder;
    /** The motor controller for steering. */
    private final PIDSparkMax steeringController;
    /** The PID controller for steering. */
    private final PIDController steeringPID;
    /** The motor controller used for driving. */
    private final PIDSparkMax driveController;

    /** The last calculated speed error. */
    private double speedError;

    /** PID P value. */
    private static final double K_P = 0.0043;
    /** PID I value. */
    private static final double K_I = 0.00;
    /** PID D value. */
    private static final double K_D = 0.0001;

    /**
     * The constructor.
     * 
     * @param moduleLocation     The physical location.
     * @param steeringEncoder    The steering encoder.
     * @param steeringController The steering motor controller.
     * @param driveController    The drive motor controller.
     */
    protected SDSSwerveModule(final Translation2d moduleLocation, final CANCoder steeringEncoder,
            final PIDSparkMax steeringController, final PIDSparkMax driveController, final Configuration conf) {
        this(moduleLocation, steeringEncoder, steeringController, driveController, conf,
                new PIDController(K_P, K_I, K_D));
    }

    /**
     * The constructor.
     * 
     * @param moduleLocation     The physical location.
     * @param steeringEncoder    The steering encoder.
     * @param steeringController The steering motor controller.
     * @param driveController    The drive motor controller.
     */
    protected SDSSwerveModule(final Translation2d moduleLocation, final CANCoder steeringEncoder,
            final PIDSparkMax steeringController, final PIDSparkMax driveController,
            final Configuration conf, final PIDController pid) {
        this.location = moduleLocation;
        this.steeringEncoder = steeringEncoder;
        this.steeringController = steeringController;
        this.driveController = configureDriveMotor(driveController, conf);
        this.steeringPID = pid;
        this.steeringPID.enableContinuousInput(-180, 180);
    }

    /**
     * Get the steering motor controller.
     * 
     * @return The controller object.
     */
    public PIDSparkMax getSteeringController() {
        return steeringController;
    }

    /**
     * Get the Disney motor controller.
     * 
     * @return The controller object.
     */
    public PIDSparkMax getDriveController() {
        return driveController;
    }

    /**
     * Get the module's location in relation to the center of mass of the robot.
     *
     * @return Location2d object representing the offset.
     */
    @Override
    public Translation2d getLocation() {
        return location;
    }

    /**
     * Process the desired state and set the output values for the motor
     * controllers.
     * 
     * @param desiredState The direction and speed.
     */
    @Override
    public void setDesiredState(final SwerveModuleState desiredState) {
        final SwerveModuleState state = calculateSteeringAngle(desiredState);

        // Run Steering angle PID to calculate output since the Spark Max can't take
        // advantage of the Cancoder
        final double angleOutput = steeringPID.calculate(getAngle().getDegrees(), state.angle.getDegrees());
        steeringController.set(angleOutput);

        // Set the drive motor output speed
        if (state.speedMetersPerSecond == 0) {
            driveController.getPidController().setIAccum(0);
        }
        this.speedError = state.speedMetersPerSecond - driveController.getEncoder().getRate();
        if (state.speedMetersPerSecond == 0) {
            driveController.setSetpoint(state.speedMetersPerSecond);
        }
    }

    /**
     * Returns the current angle of the swerve module.
     *
     * @return The current angle of the module.
     */
    private Rotation2d getAngle() {
        return Rotation2d.fromDegrees(steeringEncoder.getAbsolutePosition());
    }

    /**
     * Optimizes the desired module angle by taking into account the current module
     * angle.
     *
     * @param desiredState The module state as calculated by a SwerveDriveKinematics
     *                     object.
     * @return The optimized module state.
     */
    private SwerveModuleState calculateSteeringAngle(final SwerveModuleState desiredState) {
        return SwerveModuleState.optimize(desiredState, getAngle());
    }

    /**
     * Configures a PIDSparkMax for use as the drive motor on a MK3 swerve module.
     *
     * @param motor Drive motor controller to configure.
     * @return Drive motor controller for chaining.
     */
    private static PIDSparkMax configureDriveMotor(final PIDSparkMax motor, final Configuration conf) {
        // Get raw objects from the PIDSparkMax
        final CANSparkMax sparkMax = motor.getMotorController();
        final RelativeEncoder encoder = motor.getEncoder().getRaw();
        final SparkMaxPIDController pid = motor.getPidController();

        // Set Motor controller configuration
        motor.setControlType(PIDControlType.Velocity);
        sparkMax.setIdleMode(CANSparkMax.IdleMode.kBrake);
        // Set velocity conversion to convert RPM to M/s
        encoder.setVelocityConversionFactor(conf.getConversion() / 60.0);
        // Set Position conversion to convert from Rotations to M
        encoder.setPositionConversionFactor(conf.getConversion());

        // Configure PID
        // https://docs.revrobotics.com/sparkmax/operating-modes/closed-loop-control
        pid.setP(0.0);
        pid.setI(0.00015);
        pid.setD(0.0);
        pid.setFF(0.219);

        // Return the original object so this can be chained
        return motor;
    }

    @Override
    public void initSendable(final SendableBuilder builder) {
        builder.setActuator(true);
        builder.addDoubleProperty("Angle Error", steeringPID::getPositionError, null);
        builder.addDoubleProperty("Speed Error", () -> speedError, null);
        builder.addDoubleProperty("Speed", () -> driveController.getEncoder().getRate(), null);
    }
}
