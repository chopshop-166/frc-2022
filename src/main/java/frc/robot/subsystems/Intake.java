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

    private static final double ROLLER_SPEED = 0.5;
    private static final double DEPLOY_EXTEND_SPEED = 0.25;
    private static final double DEPLOY_RETRACT_SPEED = -0.25;

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
        return cmd("Extend Intake").onExecute(() -> {
            // Using validators in a modifier in combination with using it to stop the
            // command
            deploymentMotor.set(limit.applyAsDouble(DEPLOY_EXTEND_SPEED));
        }).until(deploymentMotor::errored).onEnd((interrupted) -> {
            deploymentMotor.set(0.0);
            rollerMotor.set(rollerDirection.get(ROLLER_SPEED));

        });
    }

    public CommandBase rollIntake(SpinDirection rollerDirection) {
        return startEnd("Start Intake", () -> {
            rollerMotor.set(rollerDirection.get(ROLLER_SPEED));
        }, () -> rollerMotor.stopMotor());
    }

    public CommandBase startRoller(SpinDirection rollerDirection) {
        return instant("Start Roller", () -> {
            rollerMotor.set(rollerDirection.get(ROLLER_SPEED));
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
        }).until(deploymentMotor::errored).onEnd((interrupted) -> {
            deploymentMotor.set(0.0);
        });
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    @Override
    public void safeState() {
        rollerMotor.stopMotor();
        deploymentMotor.stopMotor();
    }
}