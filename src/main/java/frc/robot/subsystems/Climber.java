package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.chopshop166.chopshoplib.commands.SmartSubsystemBase;
import com.chopshop166.chopshoplib.motors.Modifier;
import com.chopshop166.chopshoplib.motors.SmartMotorController;
import com.chopshop166.chopshoplib.states.SpinDirection;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.maps.subsystems.ClimberMap;

public class Climber extends SmartSubsystemBase {

    // Constants:

    private static final double ROTATE_SPEED = 0.2;

    private final SmartMotorController extendMotor;
    private final SmartMotorController rotateMotor;

    private final Modifier extendLimit;
    private final Modifier rotateLimit;

    private final DoubleSupplier extendCurrent;
    private final DoubleSupplier rotateCurrent;

    public enum ExtendDirection {
        EXTEND(0.2), RETRACT(-0.2);

        private final double direction;

        private ExtendDirection(double direction) {
            this.direction = direction;
        }

        public double get() {
            return direction;
        }
    }

    public Climber(ClimberMap map) {

        extendMotor = map.getExtendMotor();
        rotateMotor = map.getRotateMotor();
        extendCurrent = map.getExtendCurrent();
        rotateCurrent = map.getRotateCurrent();

        extendLimit = Modifier.unless(extendMotor::errored);
        rotateLimit = Modifier.unless(rotateMotor::errored);

    }

    // Move the motor based off a variable speed
    public CommandBase extendSpeed(DoubleSupplier speed) {
        return cmd("Extend Speed").onExecute(() -> {
            extendMotor.set(extendLimit.applyAsDouble(speed.getAsDouble()));
            SmartDashboard.putNumber("Climber Speed", extendMotor.get());
        }).runsUntil(extendMotor::errored).onEnd((interrupted) -> {
            extendMotor.set(0.0);
        });
    }

    public CommandBase climb(DoubleSupplier extendSpeed, DoubleSupplier rotateSpeed) {
        return cmd("Extend Speed").onExecute(() -> {
            extendMotor.set(extendLimit.applyAsDouble(extendSpeed.getAsDouble()));
            rotateMotor.set(extendLimit.applyAsDouble(-rotateSpeed.getAsDouble() * 0.25));
        }).runsUntil(() -> extendMotor.errored() || rotateMotor.errored()).onEnd((interrupted) -> {
            extendMotor.set(0.0);
            rotateMotor.set(0.0);
        });

    }

    public CommandBase rotateSpeed(DoubleSupplier speed) {
        return cmd("Rotate Speed").onExecute(() -> {
            rotateMotor.set(rotateLimit.applyAsDouble(speed.getAsDouble()));
            SmartDashboard.putNumber("Rotate Speed", rotateMotor.get());
        }).runsUntil(rotateMotor::errored).onEnd((interrupted) -> {
            rotateMotor.set(0.0);
        });
    }

    public CommandBase rotate(SpinDirection direction) {
        return cmd("Rotate").onExecute(() -> {
            rotateMotor.set(rotateLimit.applyAsDouble(direction.apply(ROTATE_SPEED)));
        }).runsUntil(rotateMotor::errored).onEnd((interrupted) -> {
            rotateMotor.set(0.0);
        });
    }

    public CommandBase extend(ExtendDirection direction) {
        return cmd("Extend").onExecute(() -> {
            extendMotor.set(extendLimit.applyAsDouble(direction.get()));
        }).runsUntil(extendMotor::errored).onEnd((interrupted) -> {
            extendMotor.set(0.0);
        });
    }

    public CommandBase autoClimb() {
        // This assumes that the extending arms are fully extended over the bar and in
        // place
        return sequence("Automatic Climb",
                // Move rotating arms out of the way
                rotate(SpinDirection.CLOCKWISE),

                // Pull the robot up
                extend(ExtendDirection.RETRACT),

                // Get the rotating arms in place
                rotate(SpinDirection.COUNTERCLOCKWISE),

                // Move robot down, then extend arms slightly ouy
                // Extend slightly out

                // Rotate the robot cw
                // Rotate slightly ccw

                // Extend arm to reach next bar
                extend(ExtendDirection.EXTEND),

                // Rotate robot so that extending arm is in place
                rotate(SpinDirection.CLOCKWISE),

                // Pull robot into the next bar
                extend(ExtendDirection.RETRACT)

        // Give rotating arms room to move out of the way
        // Extend slightly out

        );
    }

    @Override
    public void safeState() {
        extendMotor.set(0.0);
        rotateMotor.set(0.0);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Climber Extend Current Draw", extendCurrent.getAsDouble());
        SmartDashboard.putNumber("Climber Rotate Current Draw", rotateCurrent.getAsDouble());
    }
}
