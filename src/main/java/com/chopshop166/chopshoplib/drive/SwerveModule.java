package com.chopshop166.chopshoplib.drive;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.Sendable;

/**
 * Base interface for a swerve module.
 * 
 * Contains information about rotation and velocity.
 */
public interface SwerveModule extends Sendable {

    /**
     * Get the modules location in relation to the CoM of the robot.
     *
     * @return Location2d object representing the offset
     */
    Translation2d getLocation();

    /**
     * Process the desired state and set the output values for the motor
     * controllers.
     * 
     * @param desiredState The direction and speed.
     */
    void setDesiredState(final SwerveModuleState desiredState);
}
