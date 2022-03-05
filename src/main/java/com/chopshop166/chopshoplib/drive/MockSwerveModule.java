package com.chopshop166.chopshoplib.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.SendableBuilder;

public class MockSwerveModule implements SwerveModule {

    private final Translation2d location;
    private SwerveModuleState desiredState = new SwerveModuleState();

    /**
     * @param location Location of the module
     */
    public MockSwerveModule(final Translation2d location) {
        this.location = location;
    }

    /**
     * Get the modules location in relation to the CoM of the robot.
     *
     * @return Location2d object representing the offset
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
        this.desiredState = desiredState;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        // TODO Auto-generated method stub
    }

    @Override
    public double getDistance() {
        return 0;
    }

    @Override
    public void resetDistance() {

    }

    @Override
    public Rotation2d getAngle() {
        return desiredState.angle;
    }

    @Override
    public SwerveModuleState getState() {
        return new SwerveModuleState();
    }
}
