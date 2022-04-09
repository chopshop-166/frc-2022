package frc.robot.subsystems;

import java.util.Map;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.chopshop166.chopshoplib.PersistenceCheck;
import com.chopshop166.chopshoplib.commands.SmartSubsystemBase;
import com.chopshop166.chopshoplib.motors.SmartMotorController;
import com.chopshop166.chopshoplib.sensors.IEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.maps.subsystems.ShooterMap;

public class Shooter extends SmartSubsystemBase {

    private final SmartMotorController motor;
    private final IEncoder encoder;

    private static final double MAX_RPM = 5300;
    private static final double RPM_BUFFER = 20;

    private double shootSpeed;

    private double variableSpeed = HubSpeed.HIGH.get();

    private PIDController pid;
    private SimpleMotorFeedforward feedforward;

    public enum HubSpeed {
        LOW(1800 / 60.0), HIGH(64), LOW_HIGH_HOOD(36);

        private double speed;

        private HubSpeed(double speed) {
            this.speed = speed;
        }

        public double get() {
            return this.speed;
        }
    }

    SimpleWidget errorWidget;
    SimpleWidget targetWidget;
    SimpleWidget encoderWidget;

    SimpleWidget variableSpeedWidget;

    SimpleWidget pidWidget;
    SimpleWidget ffWidget;
    SimpleWidget pidffWidget;

    public Shooter(ShooterMap map) {
        ShuffleboardTab tab = Shuffleboard.getTab("Shooter");
        motor = map.getMotor();
        encoder = map.getEncoder();
        feedforward = map.getFeedforward();
        shootSpeed = 0;
        pid = map.getPid();

        errorWidget = tab.add("Error Amount (RPS)", 0.0).withSize(2, 1).withPosition(0, 2);
        targetWidget = tab.add("Target Speed (RPS)", 0.0).withSize(2, 1).withPosition(0, 1);
        encoderWidget = tab.add("Encoder Rate (RPS)", 0.0).withSize(2, 1).withPosition(2, 1);
        variableSpeedWidget = tab.add("Variable Speed (RPS)", HubSpeed.HIGH.get()).withPosition(2, 2)
                .withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.ofEntries(
                        Map.entry("Min", 1),
                        Map.entry("Max", 200)));

        pidWidget = tab.add("PID Calculation", 0.0).withPosition(1, 0);
        ffWidget = tab.add("FF Calculation", 0.0).withPosition(0, 0);
        pidffWidget = tab.add("PID + FF", 0.0).withPosition(2, 0);

    }

    private void calculatePid(double speed) {
        double pidv = pid.calculate(encoder.getRate());

        // We'll eventually need an acceleration
        double ffv = feedforward.calculate(speed, 0);
        pidWidget.getEntry().setNumber(pidv);
        ffWidget.getEntry().setNumber(ffv);
        pidffWidget.getEntry().setNumber(pidv + ffv);
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

    public CommandBase setTargetAndStartShooter(HubSpeed hub) {
        return instant("Set Default Speed", () -> {
            shootSpeed = hub.get();
            pid.setSetpoint(shootSpeed);
        });
    }

    public CommandBase setTargetVariable() {
        return instant("Set Default Speed", () -> {
            shootSpeed = variableSpeed;
            pid.setSetpoint(shootSpeed);
        });
    }

    public CommandBase stop() {
        return instant("Stop Shooter", this::safeState);
    }

    // Speed doesn't need to be set here, since it is already set in
    // setTargetHub/setSpeed

    public CommandBase waitUntilSpeedUp() {
        BooleanSupplier check = () -> Math.abs(encoder.getRate() - shootSpeed) < RPM_BUFFER;
        PersistenceCheck p = new PersistenceCheck(20, check);
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

        errorWidget.getEntry().setNumber(Math.abs(shootSpeed - encoder.getRate()));
        targetWidget.getEntry().setNumber(shootSpeed);
        encoderWidget.getEntry().setNumber(encoder.getRate());

        variableSpeed = variableSpeedWidget.getEntry().getDouble(HubSpeed.HIGH.get());
        calculatePid(shootSpeed);
    }

    @Override
    public void safeState() {
        motor.stopMotor();
        shootSpeed = 0.0;
        pid.setSetpoint(0.0);
    }
}
