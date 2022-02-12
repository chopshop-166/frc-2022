package frc.robot.subsystems;

import java.util.LinkedList;
import java.util.function.BooleanSupplier;

import com.chopshop166.chopshoplib.SampleBuffer;
import com.chopshop166.chopshoplib.commands.SmartSubsystemBase;
import com.chopshop166.chopshoplib.motors.Modifier;
import com.chopshop166.chopshoplib.motors.PIDControlType;
import com.chopshop166.chopshoplib.motors.SmartMotorController;
import com.chopshop166.chopshoplib.sensors.IColorSensor;
import com.chopshop166.chopshoplib.states.SpinDirection;
import com.fasterxml.jackson.databind.ser.std.StdKeySerializers.Default;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.maps.RobotMap.BallTransportMap;
// we're going to need shooter map too whenever that gets pushed to main

public class BallTransport extends SmartSubsystemBase {
    private final SmartMotorController bottomMotor;
    private final SmartMotorController topMotor;
    private final IColorSensor colorSensor;
    private final BooleanSupplier laserSwitch;

    // this is sample buffer for now, it will be until new sample buffer is merged
    // in the meantime you may cry about it
    private class SampleBuffer<E> extends LinkedList<E> {
        /** The index of the next sample to add. */
        private int sampleCap;

        /**
         * Create a Sample Buffer with a default length of 25.
         */
        public SampleBuffer() {
            this.sampleCap = 25;
        }

        /**
         * Create a Sample Buffer.
         * 
         * @param numSamples The number of samples to use.
         */
        public SampleBuffer(final int numSamples) {
            this.sampleCap = numSamples;
        }

        /**
         * Add a new sample to the buffer.
         * 
         * @param sample The value to add.
         */
        @Override
        public boolean add(E sample) {
            if (size() >= this.sampleCap) {
                removeFirst();
            }
            return super.add(sample);
        }

    }

    private final Alliance allianceColor;

    // creates a color sample buffer with a limit of 10 values.
    private SampleBuffer<Color> colorBuffer = new SampleBuffer<Color>(10);

    private final double SHUNT_SPEED = 1.0;
    private final double TRANSPORT_SPEED = 1.0;

    public BallTransport(final BallTransportMap map) {
        this.bottomMotor = map.getBottomMotor();
        this.topMotor = map.getTopMotor();
        this.colorSensor = map.getColorSensor();
        this.laserSwitch = map.getLaserSwitch();
        this.allianceColor = DriverStation.getAlliance();
    }

    public CommandBase setMotors(SpinDirection conveyorDirection) {
        return startEnd("Set Transport motors", () -> {
            topMotor.set(conveyorDirection.get(TRANSPORT_SPEED));
            bottomMotor.set(conveyorDirection.get(TRANSPORT_SPEED));
        }, () -> {
            topMotor.set(0);
            bottomMotor.set(0);
        });
    }

    private void updateColorBuffer() {
        colorBuffer.add(colorSensor.getColor());
    }

    @Override
    public void periodic() {
        updateColorBuffer();
    }

    @Override
    public void safeState() {
        bottomMotor.stopMotor();
        topMotor.stopMotor();
    }
}
