package frc.robot.subsystems;

import java.util.Map;
import java.util.function.BooleanSupplier;
import com.chopshop166.chopshoplib.SampleBuffer;
import com.chopshop166.chopshoplib.commands.BuildCommand;
import com.chopshop166.chopshoplib.commands.SmartSubsystemBase;
import com.chopshop166.chopshoplib.motors.SmartMotorController;
import com.chopshop166.chopshoplib.sensors.IColorSensor;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import frc.robot.Robot;
import frc.robot.maps.RobotMap.BallTransportMap;

public class BallTransport extends SmartSubsystemBase {
    private final SmartMotorController bottomMotor;
    private final SmartMotorController topMotor;
    private final IColorSensor colorSensor;
    private final BooleanSupplier laserSwitch;

    // "random values that get bigger when it's closer"
    static final private int BALL_DETECTION_LIMIT = 1000;

    private final Alliance allianceColor = DriverStation.getAlliance();

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

    private CommandBase noBall() {
        return cmd("Wait for Ball").onExecute(() -> {
            bottomMotor.set(TRANSPORT_SPEED);
            topMotor.stopMotor();
        }).until(() -> {
            return colorSensorBallLimit();
        }).onEnd(() -> {
            colorBuffer.addFirst(colorSensor.getColor());
        });
    }

    private CommandBase ballAtColor() {
        return cmd("Move ball from color sensor to laser").onExecute(() -> {
            bottomMotor.set(TRANSPORT_SPEED);
            topMotor.set(TRANSPORT_SPEED);
        }).until(() -> {
            return laserSwitch.getAsBoolean();
        });

    }

    private CommandBase ballAtLaser() {
        return cmd("Keep ball on laser switch").onExecute(() -> {
            bottomMotor.set(TRANSPORT_SPEED);
            topMotor.stopMotor();
        }).until(() -> {
            return colorSensorBallLimit();
        });
    }

    public CommandBase stopTransport() {
        return instant("Stop Ball Transport", () -> {
            bottomMotor.stopMotor();
            topMotor.stopMotor();
        });
    }

    public CommandBase loadShooter() {
        return cmd("Load Shooter").onExecute(() -> {
            topMotor.set(TRANSPORT_SPEED);
            bottomMotor.set(TRANSPORT_SPEED);
        }).onEnd(() -> {
            colorBuffer.clear();
            topMotor.stopMotor();
            bottomMotor.stopMotor();
        }).withTimeout(4);
    }

    private Command noBall = noBall();
    private Command ballAtColor = ballAtColor();
    private Command ballAtLaser = ballAtLaser();
    private Command ballAtLaserAndColor = stopTransport();

    // command selector enum used for command selector
    private enum CommandSelector {
        WAITFORBALL,
        MOVEBALLTOLASER,
        WAITFORBALLNOLASER,
        INTAKEFILLED
    }

    // returns a value of command selector enum based on sensor input.

    private CommandSelector commandSelector() {
        if (colorSensorBallLimit() && laserSwitch.getAsBoolean()) {
            return CommandSelector.WAITFORBALL;
        } else if (colorSensorBallLimit()) {
            return CommandSelector.MOVEBALLTOLASER;
        } else if (laserSwitch.getAsBoolean()) {
            return CommandSelector.WAITFORBALLNOLASER;
        } else {
            return CommandSelector.INTAKEFILLED;
        }
    }

    // Creates a map of entries for the command selector to use.
    private final Map<Object, Command> selectCommandMap = Map.ofEntries(
            Map.entry(CommandSelector.WAITFORBALL, noBall),
            Map.entry(CommandSelector.MOVEBALLTOLASER, ballAtLaserAndColor),
            Map.entry(CommandSelector.WAITFORBALLNOLASER, ballAtColor),
            Map.entry(CommandSelector.INTAKEFILLED, ballAtLaser));

    // select command that determines what command needs to be run based on the
    // command selector.
    public SelectCommand loadCargoWithIntake() {
        return new SelectCommand(selectCommandMap, this::commandSelector);
    }

    // Unloads cargo that is opposite of the alliance color
    // Intake must be deployed backwards for this
    public CommandBase removeCargo() {
        return cmd("Remove Cargo").onExecute(() -> {
            switch (allianceColor) {
                default:
                    if (colorBuffer.isEmpty()) {
                        // this accounts for if the color buffer is empty
                        break;
                    } else if (colorBuffer.size() == 1) {
                        // this accounts for if there's only one ball in the color buffer
                        topMotor.set(REMOVE_SPEED);
                        bottomMotor.set(REMOVE_SPEED);
                        colorBuffer.clear();
                        break;
                    }
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
                    break;
            }
        }).until(() -> {
            return !colorSensorBallLimit() && !laserSwitch.getAsBoolean();
        }).onEnd(() -> {
            bottomMotor.stopMotor();
            topMotor.stopMotor();
        }).withTimeout(5);
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Laser Switch Activated", laserSwitch.getAsBoolean());
        SmartDashboard.putBoolean("Cargo in Color Sensor", colorSensorBallLimit());
    }

    @Override
    public void safeState() {
        bottomMotor.stopMotor();
        topMotor.stopMotor();
    }
}
