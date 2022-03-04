package frc.robot.subsystems;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.chopshop166.chopshoplib.PersistenceCheck;
import com.chopshop166.chopshoplib.commands.SmartSubsystemBase;
import com.chopshop166.chopshoplib.motors.SmartMotorController;
import com.chopshop166.chopshoplib.sensors.IEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.maps.RobotMap.ShooterMap;

public class Shooter extends SmartSubsystemBase {

    private final SmartMotorController motor;
    private final IEncoder encoder;

    private static final double MAX_RPM = 5300;
    private static final double RPM_BUFFER = 10;
    private double shootSpeed;

    private PIDController pid = new PIDController(0.82212 / 4.0, 0, 0);
    private SimpleMotorFeedforward feedforward;

    public enum HubSpeed {
        LOW(1500 / 60.0), HIGH(2650 / 60.0);

        private double speed;

        private HubSpeed(double speed) {
            this.speed = speed;
        }

        public double get() {
            return this.speed;
        }
    }

    public Shooter(ShooterMap map) {
        motor = map.getMotor();
        encoder = map.getEncoder();
        feedforward = map.getFeedforward();
        shootSpeed = 0;
    }

    private void calculatePid(double speed) {
        double pidv = pid.calculate(encoder.getRate());

        // We'll eventually need an acceleration
        double ffv = feedforward.calculate(speed, (speed == 0) ? 0 : 0.0);
        SmartDashboard.putNumber("PID Calculation", pidv);
        SmartDashboard.putNumber("FF Calculation", ffv);
        SmartDashboard.putNumber("PID + FF", pidv + ffv);
        if (speed != 0) {
            motor.setSetpoint(pidv + ffv);
        }
    }

    public Command testSpeed(double speed) {
        return startEnd("Test Speed", () -> {
            motor.set(speed);
        }, () -> {
            motor.set(0.0);
        });
    }

    public CommandBase setTargetHub(HubSpeed hub) {
        return instant("Set Default Speed", () -> {
            shootSpeed = hub.get();
            pid.setSetpoint(shootSpeed);
        });
    }

    public CommandBase stop() {
        return instant("Stop Shooter", this::safeState);
    }

    // Speed doesn't need to be set here, since it is already set in
    // setTargetHub/setSpeed
    public CommandBase waitUntilSpeedUp() {
        BooleanSupplier check = () -> Math.abs(encoder.getRate())
                - Math.min(MAX_RPM, shootSpeed) < RPM_BUFFER;
        PersistenceCheck p = new PersistenceCheck(5, check);
        return cmd("Wait Until Speed Up").until(p);
    }

    public CommandBase setSpeed(DoubleSupplier speed) {
        return instant("Set Speed", () -> {
            double speeds = speed.getAsDouble();
            pid.setSetpoint(speeds * speeds * MAX_RPM);
        });
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Error Amount (RPM)", shootSpeed * 60.0 - encoder.getRate() * 60.0);
        SmartDashboard.putNumber("Target Speed (RPM)", shootSpeed * 60.0);
        SmartDashboard.putNumber("Encoder Rate (RPM)", encoder.getRate() * 60.0);
        calculatePid(shootSpeed);
    }

    @Override
    public void safeState() {
        motor.stopMotor();
        shootSpeed = 0.0;
        pid.setSetpoint(0.0);
    }
}
