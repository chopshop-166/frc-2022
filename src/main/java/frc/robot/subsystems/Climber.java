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

    private final DoubleSupplier gyroPitch;

    public enum ExtendDirection {
        EXTEND(1.0), RETRACT(-1.0);

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
        gyroPitch = map.getGyroPitch();

    }

    public void resetEncoders() {
        extendMotor.getEncoder().reset();
        rotateMotor.getEncoder().reset();
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
        }).onEnd((interrupted) -> {
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

    public CommandBase extendDistance(double encoderPosition) {
        return cmd("Extend Distance").onInitialize(() -> {
        }).onExecute(() -> {
            extendMotor.set(Math.signum(encoderPosition - extendMotor.getEncoder().getDistance()) * 1.0);
        }).runsUntil(() -> extendMotor.errored()
                || Math.abs(extendMotor.getEncoder().getDistance() - encoderPosition) < 0.5)
                .onEnd((interrupted) -> {
                    extendMotor.set(0.0);
                });
    }

    public CommandBase resetArms() {
        return sequence("Reset Arms",
                extend(ExtendDirection.RETRACT),
                rotate(SpinDirection.COUNTERCLOCKWISE),
                instant("Reset Encoders", () -> {
                    extendMotor.getEncoder().reset();
                    rotateMotor.getEncoder().reset();
                }));
    }

    public CommandBase rotateDistance(double encoderPosition) {
        return cmd("Rotate Distance").onInitialize(() -> {
        }).onExecute(() -> {
            rotateMotor.set(Math.signum(encoderPosition - rotateMotor.getEncoder().getDistance()) * ROTATE_SPEED);
        }).runsUntil(() -> rotateMotor.errored()
                || Math.abs(rotateMotor.getEncoder().getDistance() - encoderPosition) < 0.5)
                .onEnd((interrupted) -> {
                    rotateMotor.set(0.0);
                });
    }

    public CommandBase autoClimb() {
        // This assumes that the extending arms are fully extended over the bar and in
        // place
        return sequence("Automatic Climb",
                extendDistance(13),
                rotateDistance(4.9),
                extendDistance(146.33)/*
                                       * ,
                                       * rotateDistance(14.9),
                                       * extendDistance(459),
                                       * rotateDistance(9.6),
                                       * extendDistance(145),
                                       * rotateDistance(0),
                                       * extendDistance(0),
                                       * rotateDistance(5.28),
                                       * extendDistance(56.8)
                                       */);

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
        SmartDashboard.putNumber("Robot Pitch", Math.toDegrees(gyroPitch.getAsDouble()));
        SmartDashboard.putNumber("Climber Extend Encoder", extendMotor.getEncoder().getDistance());
        SmartDashboard.putNumber("Climber Rotate Encoder", rotateMotor.getEncoder().getDistance());
    }
}
