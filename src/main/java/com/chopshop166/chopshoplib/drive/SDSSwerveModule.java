package com.chopshop166.chopshoplib.drive;

import com.chopshop166.chopshoplib.motors.PIDSparkMax;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.SendableBuilder;

/** Swerve Drive Specialties Mk3. */
public class SDSSwerveModule implements SwerveModule {

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
    public SDSSwerveModule(final Translation2d moduleLocation, final CANCoder steeringEncoder,
            final PIDSparkMax steeringController, final PIDSparkMax driveController) {
        this(moduleLocation, steeringEncoder, steeringController, driveController, new PIDController(K_P, K_I, K_D));
    }

    /**
     * The constructor.
     * 
     * @param moduleLocation     The physical location.
     * @param steeringEncoder    The steering encoder.
     * @param steeringController The steering motor controller.
     * @param driveController    The drive motor controller.
     */
    public SDSSwerveModule(final Translation2d moduleLocation, final CANCoder steeringEncoder,
            final PIDSparkMax steeringController, final PIDSparkMax driveController, final PIDController pid) {
        this.location = moduleLocation;
        this.steeringEncoder = steeringEncoder;
        this.steeringController = steeringController;
        this.driveController = driveController;
        this.steeringPID = pid;
        this.steeringPID.enableContinuousInput(-180, 180);
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
        driveController.setSetpoint(state.speedMetersPerSecond);
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

    @Override
    public void initSendable(final SendableBuilder builder) {
        builder.setActuator(true);
        builder.addDoubleProperty("Angle Error", steeringPID::getPositionError, null);
        builder.addDoubleProperty("Speed Error", () -> speedError, null);
        builder.addDoubleProperty("Speed", () -> driveController.getEncoder().getRate(), null);
    }
}
