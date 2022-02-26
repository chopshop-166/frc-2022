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
import edu.wpi.first.wpilibj2.command.SelectCommand;
import frc.robot.maps.RobotMap.BallTransportMap;

public class BallTransport extends SmartSubsystemBase {
    private final SmartMotorController bottomMotor;
    private final SmartMotorController topMotor;
    private final IColorSensor colorSensor;
    private final BooleanSupplier laserSwitch;

    // "random values that get bigger when it's closer"
    static private final int BALL_DETECTION_LIMIT = 1000;
    static private final double REMOVE_SPEED = -0.25;
    static private final double TRANSPORT_SPEED = 0.25;

    private final Alliance allianceColor = DriverStation.getAlliance();

    // creates a color sample buffer.
    private SampleBuffer<Color> colorBuffer = new SampleBuffer<>(2);

    // command selector enum used for command selector
    private enum CommandSelector {
        LOWER_TO_COLOR,
        STOP,
        MOVE_BOTH_TO_LASER
    }

    public BallTransport(final BallTransportMap map) {
        this.bottomMotor = map.getBottomMotor();
        this.topMotor = map.getTopMotor();
        this.colorSensor = map.getColorSensor();
        this.laserSwitch = map.getLaserSwitch();
    }

    private boolean colorSensorBallLimit() {
        return colorSensor.getProximity() > BALL_DETECTION_LIMIT;
    }

    private CommandBase moveLowerToColor() {
        return cmd("Wait for Ball").onExecute(() -> {
            bottomMotor.set(TRANSPORT_SPEED);
        }).until(() -> {
            return colorSensorBallLimit();
        }).onEnd(() -> {
            bottomMotor.stopMotor();
            colorBuffer.addFirst(colorSensor.getColor());
        });
    }

    private CommandBase moveBothToLaser() {
        return cmd("Move ball from color sensor to laser").onExecute(() -> {
            bottomMotor.set(TRANSPORT_SPEED);
            topMotor.set(TRANSPORT_SPEED);
        }).until(() -> {
            return laserSwitch.getAsBoolean();
        }).onEnd(this::stop);
    }

    public CommandBase stopTransport() {
        return instant("Stop Ball Transport", this::safeState);
    }

    // Shooter should reach top speed before this is run
    public CommandBase loadShooter() {
        return cmd("Load Ball Into Shooter").onExecute(() -> {
            if (laserSwitch.getAsBoolean()) {
                topMotor.set(TRANSPORT_SPEED);
            }
        }).until(() -> {
            return !laserSwitch.getAsBoolean();
        }).onEnd((interrupted) -> {
            if (!interrupted) {
                colorBuffer.pop();
            }
            topMotor.stopMotor();
        });
    }

    // returns a value of command selector enum based on sensor input.
    private CommandSelector commandSelector() {
        if (colorSensorBallLimit() && laserSwitch.getAsBoolean()) {
            return CommandSelector.STOP;
        } else if (colorSensorBallLimit() && !laserSwitch.getAsBoolean()) {
            return CommandSelector.MOVE_BOTH_TO_LASER;
        } else {
            return CommandSelector.LOWER_TO_COLOR;
        }
    }

    // Creates a map of entries for the command selector to use.
    private final Map<Object, Command> selectCommandMap = Map.ofEntries(
            Map.entry(CommandSelector.LOWER_TO_COLOR, moveLowerToColor()),
            Map.entry(CommandSelector.STOP, stopTransport()),
            Map.entry(CommandSelector.MOVE_BOTH_TO_LASER, moveBothToLaser()));

    // select command that determines what command needs to be run based on the
    // command selector.
    // this needs to be run with intake

    // TODO this select command assumes that the command is run repeatedly through
    // while held this might need to be changed if testing fails
    public SelectCommand loadCargoWithIntake() {
        return new SelectCommand(selectCommandMap, this::commandSelector);
    }

    // Unloads cargo that is opposite of the alliance color
    // Intake must be deployed backwards for this
    public CommandBase removeCargo() {
        // this is pretty much a startend command but no reason to change it at this
        // point
        return cmd("Remove \"Wrong Colored\" Cargo").onExecute(() -> {
            switch (allianceColor) {
                case Red:
                    if (colorBuffer.size() == 1) {
                        // this accounts for if there's only one ball in the color buffer
                        // in this case the intake doesn't need to be deployed because it can just go
                        // out shooter end
                        if (colorBuffer.peekFirst() == Color.kFirstBlue) {
                            topMotor.set(REMOVE_SPEED);
                            bottomMotor.set(REMOVE_SPEED);
                            colorBuffer.clear();
                        }
                    } else {
                        if (colorBuffer.peekLast() == Color.kFirstBlue) {
                            topMotor.set(REMOVE_SPEED);
                            colorBuffer.removeLast();
                        }
                        if (colorBuffer.peekFirst() == Color.kFirstBlue) {
                            bottomMotor.set(-REMOVE_SPEED);
                            colorBuffer.removeFirst();
                        }
                    }
                    break;
                case Blue:
                    if (colorBuffer.size() == 1) {
                        if (colorBuffer.peekFirst() == Color.kFirstRed) {
                            topMotor.set(REMOVE_SPEED);
                            bottomMotor.set(REMOVE_SPEED);
                            colorBuffer.clear();
                        }
                    } else {
                        if (colorBuffer.peekLast() == Color.kFirstRed) {
                            topMotor.set(REMOVE_SPEED);
                            colorBuffer.removeLast();
                        }
                        if (colorBuffer.peekFirst() == Color.kFirstRed) {
                            bottomMotor.set(-REMOVE_SPEED);
                            colorBuffer.removeFirst();
                        }
                    }
                    break;
                case Invalid:
                default:
                    break;
            }
        }).onEnd(this::stop);
    }

    private String colorBufferConvertor(Color color) {
        if (color == Color.kFirstRed) {
            return "Red";
        } else if (color == Color.kFirstBlue) {
            return "Blue";
        } else {
            return "None";
        }
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Color Sensor Proximity", colorSensor.getProximity());
        SmartDashboard.putString("Color Buffer #1", colorBufferConvertor(colorBuffer.peekFirst()));
        SmartDashboard.putString("Color Buffer #2", colorBufferConvertor(colorBuffer.peekLast()));
        SmartDashboard.putBoolean("Laser Switch Activated", laserSwitch.getAsBoolean());
        SmartDashboard.putBoolean("Cargo in Color Sensor", colorSensorBallLimit());
    }

    @Override
    public void safeState() {
        bottomMotor.stopMotor();
        topMotor.stopMotor();
    }
}
