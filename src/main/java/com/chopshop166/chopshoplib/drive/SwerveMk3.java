package com.chopshop166.chopshoplib.drive;

import com.chopshop166.chopshoplib.motors.PIDControlType;
import com.chopshop166.chopshoplib.motors.PIDSparkMax;
import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

/** Swerve Drive Specialties Mk3. */
public class SwerveMk3 extends SDSSwerveModule {

    /** Overall gear ratio for the swerve module drive motor. */
    private static final double GEAR_RATIO = 1 / 6.86;
    /** Wheel diameter. */
    private static final double WHEEL_DIAMETER_M = Units.inchesToMeters(4);

    /**
     * The constructor.
     * 
     * @param moduleLocation     The physical location.
     * @param steeringEncoder    The steering encoder.
     * @param steeringController The steering motor controller.
     * @param driveController    The drive motor controller.
     */
    public SwerveMk3(final Translation2d moduleLocation, final CANCoder steeringEncoder,
            final PIDSparkMax steeringController, final PIDSparkMax driveController) {
        super(moduleLocation, steeringEncoder, steeringController, configureDriveMotor(driveController));
    }

    /**
     * Configures a PIDSparkMax for use as the drive motor on a MK3 swerve module.
     *
     * @param motor Drive motor controller to configure.
     * @return Drive motor controller for chaining.
     */
    private static PIDSparkMax configureDriveMotor(final PIDSparkMax motor) {
        // Get raw objects from the PIDSparkMax
        final CANSparkMax sparkMax = motor.getMotorController();
        final RelativeEncoder encoder = motor.getEncoder().getRaw();
        final SparkMaxPIDController pid = motor.getPidController();

        // Set Motor controller configuration
        motor.setControlType(PIDControlType.Velocity);
        sparkMax.setIdleMode(CANSparkMax.IdleMode.kBrake);
        // Set velocity conversion to convert RPM to M/s
        encoder.setVelocityConversionFactor((GEAR_RATIO * Math.PI * WHEEL_DIAMETER_M) / 60);
        // Set Position conversion to convert from Rotations to M
        encoder.setPositionConversionFactor(GEAR_RATIO * Math.PI * WHEEL_DIAMETER_M);

        // Configure PID
        // https://docs.revrobotics.com/sparkmax/operating-modes/closed-loop-control
        pid.setP(0.0);
        pid.setI(0.00015);
        pid.setD(0.0);
        pid.setFF(0.219);

        // Return the original object so this can be chained
        return motor;
    }
}
