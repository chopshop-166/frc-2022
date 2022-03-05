package frc.robot.maps.subsystems;

import com.chopshop166.chopshoplib.motors.SmartMotorController;
import com.chopshop166.chopshoplib.sensors.IEncoder;
import com.chopshop166.chopshoplib.sensors.MockEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;

public class ShooterMap {
    private SmartMotorController motor;
    private IEncoder encoder;
    private SimpleMotorFeedforward feedforward;
    private PIDController pid;

    public ShooterMap(SmartMotorController motor, IEncoder encoder, SimpleMotorFeedforward feedforward,
            PIDController pid) {
        this.motor = motor;
        this.encoder = encoder;
        this.feedforward = feedforward;
        this.pid = pid;
    }

    public ShooterMap() {
        this(new SmartMotorController(), new MockEncoder(), new SimpleMotorFeedforward(0.0, 0.0),
                new PIDController(0, 0, 0));

    }

    public SimpleMotorFeedforward getFeedforward() {
        return feedforward;
    }

    public SmartMotorController getMotor() {
        return motor;
    }

    public IEncoder getEncoder() {
        return encoder;
    }

    public PIDController getPid() {
        return pid;
    }

}