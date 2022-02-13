package frc.robot.subsystems;

import java.util.function.BooleanSupplier;
import com.chopshop166.chopshoplib.SampleBuffer;
import com.chopshop166.chopshoplib.commands.SmartSubsystemBase;
import com.chopshop166.chopshoplib.motors.SmartMotorController;
import com.chopshop166.chopshoplib.sensors.IColorSensor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.maps.RobotMap.BallTransportMap;

public class BallTransport extends SmartSubsystemBase {
    private final SmartMotorController bottomMotor;
    private final SmartMotorController topMotor;
    private final IColorSensor colorSensor;
    private final BooleanSupplier laserSwitch;
    private final Alliance allianceColor;

    private boolean cargoInColorSensor = false;
    private final double SHUNT_SPEED = 1.0;
    private final double TRANSPORT_SPEED = 1.0;

    // creates a color sample buffer.
    private SampleBuffer<Color> colorBuffer = new SampleBuffer<Color>(2);

    public BallTransport(final BallTransportMap map) {
        this.bottomMotor = map.getBottomMotor();
        this.topMotor = map.getTopMotor();
        this.colorSensor = map.getColorSensor();
        this.laserSwitch = map.getLaserSwitch();
        this.allianceColor = DriverStation.getAlliance();
    }

    public CommandBase loadCargo() {
        return cmd("Start Ball Transport Mechanism").onExecute(() -> {
            if (laserSwitch.getAsBoolean()) {
                topMotor.set(0);
                if (cargoInColorSensor) {
                    bottomMotor.set(0);
                } else {
                    bottomMotor.set(TRANSPORT_SPEED);
                }
            } else {
                bottomMotor.set(TRANSPORT_SPEED);
                topMotor.set(TRANSPORT_SPEED);
            }
        }).finishedWhen(() -> {
            return cargoInColorSensor && laserSwitch.getAsBoolean();
        }).onEnd(() -> {
            topMotor.set(0);
            bottomMotor.set(0);
        });
    }

    public CommandBase shuntWrongCargo() {
        return cmd("Shunt Wrong Cargo").onExecute(() -> {
            switch (allianceColor) {
                case Red:
                    if (colorBuffer.peekLast() == Color.kFirstBlue) {
                        topMotor.set(SHUNT_SPEED);
                        colorBuffer.removeLast();
                    }
                    if (colorBuffer.peekFirst() == Color.kFirstBlue) {
                        bottomMotor.set(-SHUNT_SPEED);
                        colorBuffer.removeFirst();
                    }
                    break;
                case Blue:
                    if (colorBuffer.peekLast() == Color.kFirstRed) {
                        topMotor.set(SHUNT_SPEED);
                        colorBuffer.removeLast();
                    }
                    if (colorBuffer.peekFirst() == Color.kFirstRed) {
                        bottomMotor.set(-SHUNT_SPEED);
                        colorBuffer.removeFirst();
                    }
                    break;
                case Invalid:
                default:
                    break;
            }
        }).finishedWhen(() -> {
            return !cargoInColorSensor || !laserSwitch.getAsBoolean();
        }).onEnd(() -> {
            topMotor.set(0);
            bottomMotor.set(0);
        });
    }

    private void updateColorBuffer() {
        final Color colorSensed = colorSensor.getColor();
        if (colorSensed != colorBuffer.peekFirst()) {
            if (colorSensed == Color.kFirstRed || colorSensed == Color.kFirstBlue) {
                colorBuffer.add(colorSensed);
                cargoInColorSensor = true;
            } else {
                cargoInColorSensor = false;
            }
        }

    }

    @Override
    public void periodic() {
        updateColorBuffer();
        SmartDashboard.putBoolean("Laser Switch Activated", laserSwitch.getAsBoolean());
        SmartDashboard.putBoolean("Cargo in Color Sensor", cargoInColorSensor);
    }

    @Override
    public void safeState() {
        bottomMotor.stopMotor();
        topMotor.stopMotor();
    }
}
