package frc.robot.subsystems;

import java.util.Map;
import java.util.function.DoubleSupplier;

import com.chopshop166.chopshoplib.commands.SmartSubsystemBase;
import com.chopshop166.chopshoplib.motors.Modifier;
import com.chopshop166.chopshoplib.motors.SmartMotorController;
import com.chopshop166.chopshoplib.states.SpinDirection;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import frc.robot.maps.subsystems.ClimberMap;

public class Climber extends SmartSubsystemBase {

    // Constants:

    private static final double ROTATE_SPEED = 0.2;

    private final SmartMotorController extendMotor;
    private final SmartMotorController rotateMotor;

    private final Modifier extendLimit;
    private final Modifier rotateLimit;

    private final DoubleSupplier gyroPitch;

    private final String name;

    private ClimbStep climbStep;

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

    private enum ClimbStep {
        PULL_ROBOT_UP, MOVE_ROTATE_ARMS_ON, MOVE_ARMS_UP, ROTATE_ROBOT, EXTEND_FULLY, ROTATE_TO_NEXT_BAR,
        PULL_ROBOT_OFF, RESET_ROTATE_ARMS, PULL_ROBOT_UP_FULLY, PUT_ROTATING_ON_NEXT_BAR, EXTEND_ON_NEXT_BAR,
        DO_NOTHING;

        public ClimbStep getNextState() {
            switch (this) {
                case PULL_ROBOT_UP:
                    return MOVE_ROTATE_ARMS_ON;
                case MOVE_ROTATE_ARMS_ON:
                    return MOVE_ARMS_UP;
                case MOVE_ARMS_UP:
                    return ROTATE_ROBOT;
                case ROTATE_ROBOT:
                    return EXTEND_FULLY;
                case EXTEND_FULLY:
                    return ROTATE_TO_NEXT_BAR;
                case ROTATE_TO_NEXT_BAR:
                    return PULL_ROBOT_OFF;
                case PULL_ROBOT_OFF:
                    return RESET_ROTATE_ARMS;
                case RESET_ROTATE_ARMS:
                    return PULL_ROBOT_UP_FULLY;
                case PULL_ROBOT_UP_FULLY:
                    return PUT_ROTATING_ON_NEXT_BAR;
                case PUT_ROTATING_ON_NEXT_BAR:
                    return EXTEND_ON_NEXT_BAR;
                case EXTEND_ON_NEXT_BAR:
                    return DO_NOTHING;
                case DO_NOTHING:
                    return PULL_ROBOT_UP;
                default:
                    return DO_NOTHING;
            }
        }
    }

    public Climber(ClimberMap map, String name) {
        this.name = name;
        extendMotor = map.getExtendMotor();
        rotateMotor = map.getRotateMotor();

        extendLimit = Modifier.unless(extendMotor::errored);
        rotateLimit = Modifier.unless(rotateMotor::errored);
        gyroPitch = map.getGyroPitch();

        climbStep = ClimbStep.PULL_ROBOT_UP;

    }

    public void resetEncoders() {
        extendMotor.getEncoder().reset();
        rotateMotor.getEncoder().reset();
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

    // Move the motor based off a variable speed
    public CommandBase extendSpeed(DoubleSupplier speed) {
        return cmd("Extend Speed").onExecute(() -> {
            extendMotor.set(extendLimit.applyAsDouble(speed.getAsDouble()));
            SmartDashboard.putNumber("Climber Speed", extendMotor.get());
        }).runsUntil(extendMotor::errored).onEnd((interrupted) -> {
            extendMotor.set(0.0);
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
                || Math.abs(extendMotor.getEncoder().getDistance() - encoderPosition) < 2)
                .onEnd((interrupted) -> {
                    extendMotor.set(0.0);
                });
    }

    public CommandBase extendDistance(double encoderPosition, double speedFactor) {
        return cmd("Extend Distance").onInitialize(() -> {
        }).onExecute(() -> {
            extendMotor.set(Math.signum(encoderPosition - extendMotor.getEncoder().getDistance()) * speedFactor * 1.0);
        }).runsUntil(() -> extendMotor.errored()
                || Math.abs(extendMotor.getEncoder().getDistance() - encoderPosition) < 2)
                .onEnd((interrupted) -> {
                    extendMotor.set(0.0);
                });
    }

    public CommandBase extendStop() {
        return cmd("Extend Distance").onInitialize(() -> {
        }).onExecute(() -> {
            extendMotor.set(ExtendDirection.EXTEND.get());
        }).runsUntil(extendMotor::errored).onEnd(() -> {
            extendMotor.set(0.0);
        });

    }

    public CommandBase rotateDistance(double encoderPosition) {
        return cmd("Rotate Distance").onInitialize(() -> {
        }).onExecute(() -> {
            rotateMotor.set(Math.signum(encoderPosition - rotateMotor.getEncoder().getDistance()) * ROTATE_SPEED);
        }).runsUntil(() -> rotateMotor.errored()
                || Math.abs(rotateMotor.getEncoder().getDistance() - encoderPosition) < 2)
                .onEnd((interrupted) -> {
                    rotateMotor.set(0.0);
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

    public CommandBase autoClimb() {

        final Map<Object, Command> commands = Map.ofEntries(
                Map.entry(ClimbStep.PULL_ROBOT_UP, extendDistance(13)),
                Map.entry(ClimbStep.MOVE_ROTATE_ARMS_ON, rotateDistance(4.9)), // rotate
                Map.entry(ClimbStep.MOVE_ARMS_UP, extendDistance(146.33)),
                Map.entry(ClimbStep.ROTATE_ROBOT, rotateDistance(14.9)), // rortate
                Map.entry(ClimbStep.EXTEND_FULLY, extendStop()),
                Map.entry(ClimbStep.ROTATE_TO_NEXT_BAR, rotateDistance(9.6)), // rotate
                Map.entry(ClimbStep.PULL_ROBOT_OFF, extendDistance(145, 0.5)),
                Map.entry(ClimbStep.RESET_ROTATE_ARMS, rotateDistance(0)), // rotate
                Map.entry(ClimbStep.PULL_ROBOT_UP_FULLY, extendDistance(10)),
                Map.entry(ClimbStep.PUT_ROTATING_ON_NEXT_BAR, rotateDistance(5.28)), // rotate
                Map.entry(ClimbStep.EXTEND_ON_NEXT_BAR, extendDistance(56.8)),
                Map.entry(ClimbStep.DO_NOTHING, instant("Nothing", () -> {
                })));

        return sequence("Auto Climb", new SelectCommand(commands, () -> climbStep), instant("Next Step", () -> {
            climbStep = climbStep.getNextState();
        }));
    }

    public CommandBase autoClimbOld() {
        // This assumes that the extending arms are fully extended over the bar and in
        // place
        return sequence("Automatic Climb",
                extendDistance(13),
                rotateDistance(4.9),
                extendDistance(146.33),
                rotateDistance(14.9),
                // extendDistance(430),
                extendStop(),

                rotateDistance(9.6)
        /*
         * extendDistance(145,0.5),
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
        SmartDashboard.putNumber("Robot Pitch", Math.toDegrees(gyroPitch.getAsDouble()));
        SmartDashboard.putNumber("Climber Extend Encoder", extendMotor.getEncoder().getDistance());
        SmartDashboard.putNumber("Climber Rotate Encoder", rotateMotor.getEncoder().getDistance());
        SmartDashboard.putData(name + " Extend", extendMotor);
        SmartDashboard.putData(name + " Rotate", rotateMotor);
    }
}
