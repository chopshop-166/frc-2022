package frc.robot.subsystems;

import java.util.Map;
import java.util.function.BooleanSupplier;
import com.chopshop166.chopshoplib.SampleBuffer;
import com.chopshop166.chopshoplib.commands.SmartSubsystemBase;
import com.chopshop166.chopshoplib.motors.SmartMotorController;
import com.chopshop166.chopshoplib.sensors.IColorSensor;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandGroupBase;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import frc.robot.maps.RobotMap.BallTransportMap;

public class BallTransport extends SmartSubsystemBase {
    private final SmartMotorController bottomMotor;
    private final SmartMotorController topMotor;
    private final IColorSensor colorSensor;
    private final BooleanSupplier laserSwitch;

    static final private int BALL_DETECTION_LIMIT = 1000;

    private final double REMOVE_SPEED = 1.0;
    private final double TRANSPORT_SPEED = 1.0;

    // creates a color sample buffer.
    private SampleBuffer<Color> colorBuffer = new SampleBuffer<>(2);

    public BallTransport(final BallTransportMap map) {
        this.bottomMotor = map.getBottomMotor();
        this.topMotor = map.getTopMotor();
        this.colorSensor = map.getColorSensor();
        this.laserSwitch = map.getLaserSwitch();
    }

    private boolean colorSensorBallLimit() {
        return colorSensor.getProximity() < BALL_DETECTION_LIMIT;
    }

    public CommandBase runMotors(Boolean bottomMotor, Boolean topMotor) {
        return instant("Run Motors", () -> {
            if (bottomMotor) {
                this.bottomMotor.set(TRANSPORT_SPEED);
            } else {
                this.bottomMotor.set(0);
            }
            if (topMotor) {
                this.topMotor.set(TRANSPORT_SPEED);
            } else {
                this.topMotor.set(0);
            }
        });
    }

    /*
     * private ConditionalCommand ballZeroOne(){
     * return new ConditionalCommand(runMotors(true, true), runMotors(true, false) ,
     * () -> {
     * return colorSensorBallLimit() && !laserSwitch.getAsBoolean();
     * });
     * }
     * 
     * private ConditionalCommand ballZeroZero() {
     * return new ConditionalCommand(runMotors(false, false),runMotors(true, false)
     * , () -> {
     * return colorSensorBallLimit() && laserSwitch.getAsBoolean();
     * });
     * }
     * public CommandBase transportLogic() {
     * return sequence("Transport Logic", ballZeroZero(), ballZeroOne());
     * }
     */
    private enum CommandSelector {
        COLORZEROLASERZERO,
        COLORONELASERZERO,
        COLORZEROLASERONE,
        COLORONELASERONE
    }

    private CommandSelector selectSelector() {
        return () -> {

        }
    }

    public SelectCommand loadCargoWithIntake() {
        return new SelectCommand(

                Map.ofEntries(
                        Map.entry(CommandSelector.COLORZEROLASERZERO, v),
                        Map.entry(CommandSelector.COLORONELASERZERO, v),
                        Map.entry(CommandSelector.COLORZEROLASERONE, v),
                        Map.entry(CommandSelector.COLORONELASERONE, v)),
                selectSelector());
    }
    /*
     * public CommandBase loadCargoWithIntake(boolean isIntakeRunning) {
     * return cmd("Start Ball Transport Mechanism").onExecute(() -> {
     * if (isIntakeRunning) {
     * if (laserSwitch.getAsBoolean() && colorSensorBallLimit()) {
     * runMotors(false, false);
     * } else if (colorSensorBallLimit()) {
     * colorSensorToLaser();
     * } else {
     * runMotors(true, false);
     * }
     * } else {
     * if (colorSensorBallLimit() && !laserSwitch.getAsBoolean()) {
     * runMotors(true, true);
     * } else {
     * runMotors(false, false);
     * }
     * }
     * }).onEnd(this::stop);
     * }
     */

    // Shooter should reach top speed before this is run
    public CommandBase loadShooter() {
        return cmd("Load Ball Into Shooter").onExecute(() -> {
            if (laserSwitch.getAsBoolean()) {
                topMotor.set(TRANSPORT_SPEED);
            }
        }).until(() -> {
            return !laserSwitch.getAsBoolean();
        }).onEnd(() -> {
            colorBuffer.removeLast();
            topMotor.stopMotor();
        });
    }

    // Moves a ball from the color sensor to the laser switch by the shooter
    public CommandBase colorSensorToLaser() {
        return cmd("Move Ball From Color Sensor To Laser Switch").onExecute(() -> {
            if (colorSensorBallLimit() && !laserSwitch.getAsBoolean()) {
                bottomMotor.set(TRANSPORT_SPEED);
                topMotor.set(TRANSPORT_SPEED);
            }
        }).until(() -> {
            return laserSwitch.getAsBoolean();
        }).onEnd(this::stop);
    }

    // Load ball until it hits the color sensor
    @Deprecated
    public CommandBase loadBottomTransporter() {
        return cmd("Load Ball Until Color Sensor").onExecute(() -> {
            if (colorSensorBallLimit()) {
                bottomMotor.set(TRANSPORT_SPEED);
            }
        }).until(() -> {
            return colorSensorBallLimit();
        }).onEnd(() -> {
            bottomMotor.stopMotor();
        });
    }

    // TODO Needs to be fixed
    // Unloads cargo that is opposite of the alliance color
    // Intake must be deployed backwards for this
    public CommandBase removeCargo() {
        return cmd("Remove Cargo").onExecute(() -> {
            final Alliance allianceColor = DriverStation.getAlliance();
            switch (allianceColor) {
                case Red:
                    if (colorBuffer.peekLast() == Color.kFirstBlue) {
                        topMotor.set(REMOVE_SPEED);
                        colorBuffer.removeLast();
                    }
                    if (colorBuffer.peekFirst() == Color.kFirstBlue) {
                        bottomMotor.set(-REMOVE_SPEED);
                        colorBuffer.removeFirst();
                    }
                    break;
                case Blue:
                    if (colorBuffer.peekLast() == Color.kFirstRed) {
                        topMotor.set(REMOVE_SPEED);
                        colorBuffer.removeLast();
                    }
                    if (colorBuffer.peekFirst() == Color.kFirstRed) {
                        bottomMotor.set(-REMOVE_SPEED);
                        colorBuffer.removeFirst();
                    }
                    break;
                case Invalid:
                default:
                    break;
            }
        }).until(() -> {
            return !colorSensorBallLimit() || !laserSwitch.getAsBoolean();
        }).onEnd(this::stop);
    }

    private void updateBuffer() {
        final Color colorSensed = colorSensor.getColor();
        if (colorSensed != colorBuffer.peekFirst()) {
            if (colorSensed == Color.kFirstRed || colorSensed == Color.kFirstBlue) {
                colorBuffer.add(colorSensed);
            }
        }
    }

    @Override
    public void periodic() {
        updateBuffer();
        SmartDashboard.putBoolean("Laser Switch Activated", laserSwitch.getAsBoolean());
        SmartDashboard.putBoolean("Cargo in Color Sensor", colorSensorBallLimit());
    }

    @Override
    public void safeState() {
        bottomMotor.stopMotor();
        topMotor.stopMotor();
    }
}
