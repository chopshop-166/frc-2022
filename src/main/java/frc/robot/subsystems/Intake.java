package frc.robot.subsystems;

import com.chopshop166.chopshoplib.commands.SmartSubsystemBase;
import com.chopshop166.chopshoplib.motors.Modifier;
import com.chopshop166.chopshoplib.motors.PIDControlType;
import com.chopshop166.chopshoplib.motors.SmartMotorController;
import com.chopshop166.chopshoplib.states.SpinDirection;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.maps.subsystems.IntakeMap;

public class Intake extends SmartSubsystemBase {

    private final SmartMotorController deploymentMotor;
    private final SmartMotorController rollerMotor;

    private static final double ROLLER_SPEED = 0.75;
    private static final double DEPLOY_EXTEND_SPEED = 1.0;
    private static final double DEPLOY_RETRACT_SPEED = -1.0;

    private static final double ROLLER_THRESHOLD = 8.0;

    private final Modifier limit;

    public Intake(final IntakeMap map) {
        this.deploymentMotor = map.getDeploy();
        this.rollerMotor = map.getRoller();

        limit = Modifier.unless(deploymentMotor::errored);

        deploymentMotor.setControlType(PIDControlType.Velocity);

    }

    // rollerDirection is CLOCKWISE for the ball to go in, and COUNTERCLOCKWISE for
    // going out

    // Extend with the deployment motor and spin roller
    public CommandBase extend(SpinDirection rollerDirection) {
        return cmd("Extend Intake").onInitialize(() -> {
            deploymentMotor.getEncoder().reset();
        }).onExecute(() -> {
            // Using validators in a modifier in combination with using it to stop the
            // command
            deploymentMotor.set(limit.applyAsDouble(DEPLOY_EXTEND_SPEED));
            if (deploymentMotor.getEncoder().getDistance() >= ROLLER_THRESHOLD) {
                rollerMotor.set(rollerDirection.apply(ROLLER_SPEED));
            }
        }).runsUntil(deploymentMotor::errored).onEnd((interrupted) -> {
            deploymentMotor.set(0.0);
            // rollerMotor.set(rollerDirection.apply(ROLLER_SPEED));
        });
    }

    public CommandBase rollIntake(SpinDirection rollerDirection) {
        return startEnd("Start Intake", () -> {
            rollerMotor.set(rollerDirection.apply(ROLLER_SPEED));
        }, () -> rollerMotor.stopMotor());
    }

    public CommandBase startRoller(SpinDirection rollerDirection) {
        return instant("Start Roller", () -> {
            rollerMotor.set(rollerDirection.apply(ROLLER_SPEED));
        });
    }

    public CommandBase stopRoller() {
        return instant("Stop Roller", () -> {
            rollerMotor.stopMotor();
        });
    }

    // Retract with the deployment motor and stop roller
    public CommandBase retract() {
        return cmd("Retract Intake").onInitialize(() -> {
            rollerMotor.set(0.0);
        }).onExecute(() -> {
            deploymentMotor.set(limit.applyAsDouble(DEPLOY_RETRACT_SPEED));
        }).runsUntil(deploymentMotor::errored).onEnd((interrupted) -> {
            deploymentMotor.set(0.0);
        });
    }

    @Override
    public void periodic() {
        // SmartDashboard.putNumber("Intake Deploy Encoder",
        // deploymentMotor.getEncoder().getDistance());
    }

    @Override
    public void safeState() {
        rollerMotor.stopMotor();
        deploymentMotor.stopMotor();
    }
}