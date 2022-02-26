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

    public boolean colorSensorBallLimit() {
        return colorSensor.getProximity() < BALL_DETECTION_LIMIT;
    }

    public boolean getLaserSwitch() {
        return laserSwitch.getAsBoolean();
    }

    // command selector enum used for command selector
    public enum CommandSelector {
        COLORZEROLASERZERO,
        COLORONELASERZERO,
        COLORZEROLASERONE,
        COLORONELASERONE
    }

    // returns a value of command selector enum based on sensor input.

    private CommandSelector commandSelector() {
        if (colorSensorBallLimit() && laserSwitch.getAsBoolean()) {
            return CommandSelector.COLORONELASERONE;
        } else if (colorSensorBallLimit()) {
            return CommandSelector.COLORONELASERZERO;
        } else if (laserSwitch.getAsBoolean()) {
            return CommandSelector.COLORZEROLASERONE;
        } else {
            return CommandSelector.COLORONELASERONE;
        }
    }

    private CommandBase noBall() {
        return cmd("Wait for Ball").onExecute(() -> {
            bottomMotor.set(TRANSPORT_SPEED);
            topMotor.stopMotor();
        }).until(() -> {
            return colorSensorBallLimit();
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
            topMotor.stopMotor();
            bottomMotor.stopMotor();
        }).withTimeout(4);
    }

    private Command noBall = noBall();
    private Command ballAtColor = ballAtColor();
    private Command ballAtLaser = ballAtLaser();
    private Command ballAtLaserAndColor = stopTransport();

    // Creates a map of entries for the command selector to use.
    private final Map<Object, Command> selectCommandMap = Map.ofEntries(
            Map.entry(CommandSelector.COLORZEROLASERZERO, noBall),
            Map.entry(CommandSelector.COLORONELASERONE, ballAtLaserAndColor),
            Map.entry(CommandSelector.COLORONELASERZERO, ballAtColor),
            Map.entry(CommandSelector.COLORZEROLASERONE, ballAtLaser));

    // select command that determines what command needs to be run based on the
    // command selector.
    public SelectCommand loadCargoWithIntake() {
        return new SelectCommand(selectCommandMap, this::commandSelector);
    }

    // TODO Needs to be fixed
    // Unloads cargo that is opposite of the alliance color
    // Intake must be deployed backwards for this
    /*
     * public CommandBase removeCargo() {
     * return cmd("Remove Cargo").onExecute(() -> {
     * final Alliance allianceColor = DriverStation.getAlliance();
     * switch (allianceColor) {
     * case Red:
     * if (colorBuffer.peekLast() == Color.kFirstBlue) {
     * topMotor.set(REMOVE_SPEED);
     * colorBuffer.removeLast();
     * }
     * if (colorBuffer.peekFirst() == Color.kFirstBlue) {
     * bottomMotor.set(-REMOVE_SPEED);
     * colorBuffer.removeFirst();
     * }
     * break;
     * case Blue:
     * if (colorBuffer.peekLast() == Color.kFirstRed) {
     * topMotor.set(REMOVE_SPEED);
     * colorBuffer.removeLast();
     * }
     * if (colorBuffer.peekFirst() == Color.kFirstRed) {
     * bottomMotor.set(-REMOVE_SPEED);
     * colorBuffer.removeFirst();
     * }
     * break;
     * case Invalid:
     * default:
     * break;
     * }
     * }).until(() -> {
     * return !colorSensorBallLimit() || !laserSwitch.getAsBoolean();
     * }).onEnd(this::stop);
     * }
     */
    private void updateBuffer() {
        /*
         * final Color colorSensed = colorSensor.getColor();
         * if (colorSensed != colorBuffer.peekFirst()) {
         * if (colorSensed == Color.kFirstRed || colorSensed == Color.kFirstBlue) {
         * colorBuffer.add(colorSensed);
         * }
         * }
         */
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
