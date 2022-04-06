package frc.robot.subsystems;

import java.util.EnumMap;
import java.util.Map;
import java.util.function.DoubleSupplier;

import com.chopshop166.chopshoplib.PersistenceCheck;
import com.chopshop166.chopshoplib.commands.SmartSubsystemBase;
import com.chopshop166.chopshoplib.motors.Modifier;
import com.chopshop166.chopshoplib.motors.ModifierGroup;
import com.chopshop166.chopshoplib.motors.SmartMotorController;
import com.chopshop166.chopshoplib.states.SpinDirection;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import frc.robot.maps.subsystems.ClimberMap;

public class Climber extends SmartSubsystemBase {

    private static final double EXTEND_THRESHOLD = 400.0;
    private static final double RETRACT_THRESHOLD = 20.0;
    // Constants:

    private static final double ROTATE_SPEED = 0.3;

    private final SmartMotorController extendMotor;
    private final SmartMotorController rotateMotor;

    private final ModifierGroup extendLimit;
    private final Modifier rotateLimit;

    private final DoubleSupplier gyroPitch;

    private final String name;

    private ClimbStep climbStep;

    private final ClimberSide side;

    public enum ClimberSide {
        LEFT(0), RIGHT(4);

        private final int offset;

        private ClimberSide(int offset) {
            this.offset = offset;
        }

        public int getOffset() {
            return offset;
        }
    }

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
        PULL_ROBOT_UP, MOVE_ROTATE_ARMS_ON, MOVE_ARMS_UP, ROTATE_ROBOT,
        EXTEND_TO_NEXT_BAR, EXTEND_FULLY, ROTATE_TO_NEXT_BAR,
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
                    return EXTEND_TO_NEXT_BAR;
                case EXTEND_TO_NEXT_BAR:
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
                    return MOVE_ARMS_UP;
                case EXTEND_ON_NEXT_BAR:
                    return DO_NOTHING;
                case DO_NOTHING:
                    return PULL_ROBOT_UP;
                default:
                    return DO_NOTHING;
            }
        }
    }

    private static final Map<ClimberSide, Boolean> finishedStates = new EnumMap<ClimberSide, Boolean>(
            ClimberSide.class);
    private static boolean shouldReset = false;
    private final SimpleWidget extendWidget;
    private final SimpleWidget rotateWidget;
    private final SimpleWidget pitchWidget;

    private final SimpleWidget stepWidget;
    private final SimpleWidget finishedWidget;

    public Climber(ClimberMap map, String name, ClimberSide side) {

        this.name = name;
        extendMotor = map.getExtendMotor();
        rotateMotor = map.getRotateMotor();

        this.side = side;
        finishedStates.put(this.side, false);

        extendLimit = new ModifierGroup(Modifier.unless(extendMotor::errored),
                Modifier.lowerLimit(() -> extendMotor.getEncoder().getDistance() <= 12.0));
        rotateLimit = Modifier.unless(rotateMotor::errored);
        gyroPitch = map.getGyroPitch();

        climbStep = ClimbStep.PULL_ROBOT_UP;
        ShuffleboardTab tab = Shuffleboard.getTab("Climber");

        extendWidget = tab.add(name + "Extend Encoder", 0.0).withPosition(side.getOffset(),
                0).withWidget(BuiltInWidgets.kNumberBar);
        rotateWidget = tab.add(name + "Rotate Encoder", 0.0)
                .withPosition(1 + side.getOffset(), 0).withWidget(BuiltInWidgets.kDial);

        stepWidget = tab.add(name + " Step", "").withPosition(side.getOffset(), 1);
        finishedWidget = tab.add(name + " Finished?", false)
                .withPosition(1 + side.getOffset(), 1);
        pitchWidget = tab.add(name + " Robot Pitch", 0.0).withPosition(1 + side.getOffset(),
                0).withWidget(BuiltInWidgets.kDial);

        tab.add(name + " Extend", extendMotor).withPosition(side.getOffset(), 2);
        tab.add(name + " Rotate", rotateMotor).withPosition(side.getOffset(), 3);

    }

    private double limitSpeed(double speed) {
        return ((extendMotor.getEncoder().getDistance() >= EXTEND_THRESHOLD
                && extendMotor.get() > 0)
                || (extendMotor.getEncoder().getDistance() <= RETRACT_THRESHOLD && extendMotor.get() < 0))
                        ? (speed / 10.0)
                        : (speed);
    }

    public void resetSteps() {
        climbStep = ClimbStep.DO_NOTHING;
    }

    public void resetEncoders() {
        extendMotor.getEncoder().reset();
        rotateMotor.getEncoder().reset();
    }

    public CommandBase climb(DoubleSupplier extendSpeed, DoubleSupplier rotateSpeed) {

        return cmd("Extend Speed").onExecute(() -> {
            extendMotor.set(limitSpeed(extendSpeed.getAsDouble()));
            rotateMotor.set(rotateLimit.applyAsDouble(-rotateSpeed.getAsDouble() * 0.25));
        }).onEnd((interrupted) -> {
            extendMotor.set(0.0);
            rotateMotor.set(0.0);
        });

    }

    public CommandBase rotateSpeed(DoubleSupplier speed) {
        return cmd("Rotate Speed").onInitialize(() -> {

        }).onExecute(() -> {
            rotateMotor.set(rotateLimit.applyAsDouble(speed.getAsDouble()));
        }).runsUntil(rotateMotor::errored).onEnd((interrupted) -> {
            rotateMotor.set(0.0);

        });
    }

    public CommandBase extendSpeed(DoubleSupplier speed) {
        return cmd("Extend Speed").onInitialize(() -> {
        }).onExecute(() -> {
            extendMotor.set(extendLimit.applyAsDouble(speed.getAsDouble()));
        }).runsUntil(extendMotor::errored).onEnd((interrupted) -> {
            extendMotor.set(0.0);
        });
    }

    public CommandBase rotate(SpinDirection direction, double speedFactor) {
        return cmd("Rotate").onInitialize(() -> {
        }).onExecute(() -> {
            rotateMotor.set(rotateLimit.applyAsDouble(direction.apply(ROTATE_SPEED)) * speedFactor);
        }).runsUntil(rotateMotor::errored).onEnd((interrupted) -> {
            rotateMotor.set(0.0);
        });
    }

    public CommandBase extend(ExtendDirection direction, double speedFactor) {
        // Extend all the way to the hard-stop using the encoder rate to determine if
        // the motor stops spinning
        PersistenceCheck p = new PersistenceCheck(5, () -> Math.abs(extendMotor.getEncoder().getRate()) < 0.2);

        return cmd("Extend").onExecute(() -> {
            extendMotor.set(direction.get() * speedFactor);
        }).runsUntil(p).onEnd((interrupted) -> {
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

    public CommandBase extendDistanceIgnoreLimit(double encoderPosition) {
        return cmd("Extend Distance").onInitialize(() -> {

        }).onExecute(() -> {
            extendMotor.set(Math.signum(encoderPosition - extendMotor.getEncoder().getDistance()) * 1.0);
        }).runsUntil(() -> Math.abs(extendMotor.getEncoder().getDistance() - encoderPosition) < 2)
                .onEnd((interrupted) -> {
                    extendMotor.set(0.0);

                });
    }

    public CommandBase extendStop() {
        // Checks if the encoder is no longer rotating instead of current limits
        PersistenceCheck p = new PersistenceCheck(5, () -> Math.abs(extendMotor.getEncoder().getRate()) < 0.2);

        return cmd("Extend Distance").onInitialize(() -> {
        }).onExecute(() -> {
            // Safely extend
            extendMotor.set(ExtendDirection.EXTEND.get() * 0.2);
        }).runsUntil(() -> extendMotor.errored() || p.getAsBoolean()).onEnd(() -> {
            extendMotor.set(0.0);

        });

    }

    public CommandBase rotateDistance(double encoderPosition) {
        return cmd("Rotate Distance").onInitialize(() -> {
        }).onExecute(() -> {
            rotateMotor.set(Math.signum(encoderPosition - rotateMotor.getEncoder().getDistance()) * ROTATE_SPEED);
        }).runsUntil(() -> Math.abs(rotateMotor.getEncoder().getDistance() - encoderPosition) < 2)
                .onEnd((interrupted) -> {
                    rotateMotor.set(0.0);

                });
    }

    public CommandBase resetArms() {
        return sequence("Reset Arms",
                extend(ExtendDirection.RETRACT, 0.1),
                // rotate(SpinDirection.COUNTERCLOCKWISE, 0.1),
                instant("Reset Encoders", () -> {
                    extendMotor.getEncoder().reset();
                    rotateMotor.getEncoder().reset();
                }));
    }

    public CommandBase autoClimb() {

        final Map<Object, Command> commands = Map.ofEntries(
                Map.entry(ClimbStep.PULL_ROBOT_UP, extendDistance(13)),
                Map.entry(ClimbStep.MOVE_ROTATE_ARMS_ON, rotateDistance(4.9)),
                Map.entry(ClimbStep.MOVE_ARMS_UP, extendDistance(146.33)),
                Map.entry(ClimbStep.ROTATE_ROBOT, rotateDistance(14.9)),
                Map.entry(ClimbStep.EXTEND_TO_NEXT_BAR, extendDistance(400.0)),
                Map.entry(ClimbStep.EXTEND_FULLY, extendStop()),
                Map.entry(ClimbStep.ROTATE_TO_NEXT_BAR, rotateDistance(9.6)),
                Map.entry(ClimbStep.PULL_ROBOT_OFF, extendDistance(194, 0.75)),
                Map.entry(ClimbStep.RESET_ROTATE_ARMS, rotateDistance(0)),
                Map.entry(ClimbStep.PULL_ROBOT_UP_FULLY, extendDistance(10)),
                Map.entry(ClimbStep.PUT_ROTATING_ON_NEXT_BAR, rotateDistance(5.28)),
                Map.entry(ClimbStep.EXTEND_ON_NEXT_BAR, extendDistance(56.8)),
                Map.entry(ClimbStep.DO_NOTHING, instant("Nothing", () -> {
                })));

        // Check if both arms have finished before moving on to the next step
        return sequence("Auto Climb", new SelectCommand(commands, () -> climbStep),
                cmd("Increment").onInitialize(() -> {
                    finishedStates.put(side, true);
                    System.out.println(name + " running");
                    shouldReset = false;
                }).runsUntil(() -> (finishedStates.get(ClimberSide.LEFT) &&
                        finishedStates.get(ClimberSide.RIGHT)) || shouldReset)
                        .onEnd((interrupted) -> {
                            if (!interrupted) {
                                finishedStates.put(side, false);
                                shouldReset = true;
                                System.out.println(name + " finished on " + side.name());
                                climbStep = climbStep.getNextState();
                            }
                        }));

    }

    public CommandBase resetSequence() {
        return instant("Reset Sequence", () -> {
            climbStep = ClimbStep.MOVE_ARMS_UP;
        });
    }

    @Override
    public void safeState() {
        extendMotor.set(0.0);
        rotateMotor.set(0.0);
    }

    @Override
    public void periodic() {

        pitchWidget.getEntry().setNumber(Math.toDegrees(gyroPitch.getAsDouble()));
        extendWidget.getEntry().setNumber(extendMotor.getEncoder().getDistance());
        rotateWidget.getEntry().setNumber(rotateMotor.getEncoder().getDistance());
        stepWidget.getEntry().setString(climbStep.name());
        finishedWidget.getEntry().setBoolean(finishedStates.get(side));
    }
}
