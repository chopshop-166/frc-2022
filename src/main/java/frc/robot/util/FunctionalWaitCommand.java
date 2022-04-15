// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.util;

import java.util.function.DoubleSupplier;

import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * A command that does nothing but takes a specified amount of time to finish.
 * Useful for
 * CommandGroups. Can also be subclassed to make a command with an internal
 * timer.
 *
 * <p>
 * This class is provided by the NewCommands VendorDep
 */
public class FunctionalWaitCommand extends CommandBase {
    protected Timer m_timer = new Timer();
    private double m_duration;
    private final DoubleSupplier durationSupplier;

    /**
     * Creates a new WaitCommand. This command will do nothing, and end after the
     * specified duration.
     *
     * @param seconds the time to wait, in seconds
     */
    public FunctionalWaitCommand(DoubleSupplier duration) {
        durationSupplier = duration;
        SendableRegistry.setName(this, getName());
    }

    @Override
    public void initialize() {
        m_timer.reset();
        m_timer.start();
        m_duration = durationSupplier.getAsDouble();
    }

    @Override
    public void end(boolean interrupted) {
        m_timer.stop();
    }

    @Override
    public boolean isFinished() {
        return m_timer.hasElapsed(m_duration);
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }
}
