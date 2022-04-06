package frc.robot;

import java.util.function.DoubleSupplier;
import java.util.function.DoubleUnaryOperator;
import java.util.stream.Stream;

import com.chopshop166.chopshoplib.Autonomous;
import com.chopshop166.chopshoplib.commands.CommandRobot;
import com.chopshop166.chopshoplib.commands.SmartSubsystem;
import com.chopshop166.chopshoplib.controls.ButtonXboxController;
import com.chopshop166.chopshoplib.controls.ButtonXboxController.POVDirection;
import com.chopshop166.chopshoplib.states.SpinDirection;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.maps.RobotMap;
import frc.robot.subsystems.BallTransport;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Climber.ClimberSide;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Led;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Shooter.HubSpeed;
import frc.robot.util.LightAnimation;

public class Robot extends CommandRobot {

    private final ButtonXboxController driveController = new ButtonXboxController(0);
    private final ButtonXboxController copilotController = new ButtonXboxController(1);

    // private UsbCamera camera = CameraServer.startAutomaticCapture();
    private final RobotMap map = getRobotMap(RobotMap.class, "frc.robot.maps", new RobotMap());

    private final Drive drive = new Drive(map.getSwerveDriveMap());

    private final Intake intake = new Intake(map.getIntakeMap());

    private final BallTransport ballTransport = new BallTransport(map.getBallTransportMap());

    private final Led led = new Led(map.getLedMap());

    private final Shooter shooter = new Shooter(map.getShooterMap());
    private final Climber leftClimber = new Climber(map.getLeftClimberMap(), "Left", ClimberSide.LEFT);
    private final Climber rightClimber = new Climber(map.getRightClimberMap(), "Right", ClimberSide.RIGHT);

    private final LightAnimation teamColors = new LightAnimation("rotate.json", "Team Colors");

    private CommandBase shootHigh() {
        return sequence("Shoot High", shooter.setTargetAndStartShooter(HubSpeed.HIGH),
                shooter.waitUntilSpeedUp(),
                ballTransport.loadShooter(), ballTransport.moveBothMotorsToLaser(), new WaitCommand(0.15));
    }

    private CommandBase shootLow() {
        return sequence("Shoot Low", shooter.setTargetAndStartShooter(HubSpeed.LOW_HIGH_HOOD),
                shooter.waitUntilSpeedUp(),
                ballTransport.loadShooter(), ballTransport.moveBothMotorsToLaser());
    }

    @Override
    public void teleopInit() {
        leftClimber.resetEncoders();
        rightClimber.resetEncoders();
        leftClimber.resetSteps();
        rightClimber.resetSteps();
        SmartDashboard.putNumber("High Goal Speed", HubSpeed.LOW.get());

    }

    public CommandBase shootOneBallAuto() {
        return shootHigh().withName("Shoot One Ball Auto");
    }

    public CommandBase shootTwoBallsAuto() {
        return sequence("Shoot Two Balls Auto", shootHigh(),
                ballTransport.loadShooter());
    }

    public CommandBase intakeOneBallAuto() {
        return sequence("Intake One Ball",
                intake.extend(SpinDirection.COUNTERCLOCKWISE), ballTransport.loadCargoWithIntake(),
                intake.retract());
    }

    public CommandBase stopShooter() {
        return sequence("Stop Shooter", new WaitCommand(1), shooter.stop());
    }

    // Starting Against the right side of the hub, Shoot One Ball, Pickup 2 balls,
    // shoot them
    // public CommandBase threeRightAuto() {
    // return sequence("Three Ball Right Auto",
    // shootOneBallAuto(),
    // parallel("Stop Shooter", stopShooter(),
    // drive.auto(AutoPaths.threeBallRightOne)),
    // intakeOneBallAuto(), drive.auto(AutoPaths.threeBallRightTwo),
    // intakeOneBallAuto(), drive.auto(AutoPaths.threeBallRightThree),
    // shootTwoBallsAuto(), stopShooter());
    // }

    // public CommandBase twoRightAuto() {
    // return sequence("Two Ball Right Auto", shootOneBallAuto(),
    // parallel("Stop Shooter", stopShooter(), drive.auto(
    // AutoPaths.twoBallRightOne)),
    // intakeOneBallAuto(),
    // drive.auto(
    // AutoPaths.twoBallRightTwo),
    // shootOneBallAuto(),
    // parallel("Stop Shooter", stopShooter(),
    // drive.auto(AutoPaths.twoBallRightOne)));
    // }

    // public CommandBase twoLeftAuto() {
    // return sequence("Two Ball Left Auto",
    // drive.resetAuto(AutoPaths.twoBallLeftOne),
    // parallel("Stop Shooter", stopShooter(), drive.auto(
    // AutoPaths.twoBallLeftOne)),
    // intakeOneBallAuto(), drive.auto(AutoPaths.twoBallLeftTwo),
    // shootOneBallAuto(), parallel("Stop Shooter", stopShooter()));
    // }

    // // Shoot One ball and taxi
    // public CommandBase oneBallAuto() {
    // return sequence("One Ball Auto", shootOneBallAuto(),
    // drive.auto(AutoPaths.oneBallLeftOne));
    // }

    // private CommandBase weekTwoAutoHigh() {
    // return sequence("Week Two Auto High",
    // shootHigh(),

    // parallel("Stop and drive",
    // sequence("Stop shooter", new WaitCommand(2), shooter.stop()),
    // drive.driveDistance(2.8, 0, 0.5)),
    // parallel("Reset Arms", leftClimber.resetArms(), rightClimber.resetArms()));
    // }

    // private CommandBase delayedAuto() {
    // return sequence("Delayed Auto",
    // new WaitCommand(2),
    // shootHigh(),

    // parallel("Stop and drive",
    // sequence("Stop shooter", new WaitCommand(2), shooter.stop()),
    // drive.driveDistance(2.8, 0, 0.5)),
    // parallel("Reset Arms", leftClimber.resetArms(), rightClimber.resetArms()));
    // }

    // private CommandBase weekTwoAutoLow() {
    // return sequence("Week Two Auto Low",
    // shootLow(),

    // parallel("Stop and drive",
    // sequence("Stop shooter", new WaitCommand(2), shooter.stop()),
    // drive.driveDistance(2.8, 0, 0.5)),
    // parallel("Reset Arms", leftClimber.resetArms(), rightClimber.resetArms()));
    // }

    // private CommandBase onlyShoot() {
    // return sequence("Only Shoot", shootHigh());
    // }

    // @Autonomous
    // public CommandBase threeRightAuto = threeRightAuto();
    // @Autonomous
    // public CommandBase twoRightAuto = twoRightAuto();
    // @Autonomous
    // public CommandBase twoLeftAuto = twoLeftAuto();
    // @Autonomous
    // public CommandBase oneBallAuto = oneBallAuto();
    // @Autonomous
    // public CommandBase weekTwoAutoLow = weekTwoAutoLow();
    // @Autonomous
    // public CommandBase weekTwoAutoHigh = weekTwoAutoHigh();
    // @Autonomous
    // public CommandBase onlyShoot = onlyShoot();
    // @Autonomous(defaultAuto = true)
    // public CommandBase delayedAuto = delayedAuto();

    public DoubleUnaryOperator scalingDeadband(double range) {
        return speed -> {
            if (Math.abs(speed) < range) {
                return 0.0;
            } else {
                return (speed - (Math.signum(speed) * range)) / (1.0 - range);
            }
        };
    }

    public DoubleSupplier deadbandAxis(double range, DoubleSupplier axis) {
        DoubleUnaryOperator deadband = scalingDeadband(range);
        return () -> deadband.applyAsDouble(axis.getAsDouble());
    }

    @Override
    public void robotInit() {
        super.robotInit();
        // Shuffleboard.getTab("Camera").add("USB Camera 0", camera);

    }

    @Override
    public void configureButtonBindings() {

        new Trigger(DriverStation::isDSAttached).whileActiveOnce(led.serialPortSend());

        driveController.start().whenPressed(drive.resetGyro());

        copilotController.getPovButton(POVDirection.UP).whileHeld(ballTransport.runForwards());
        copilotController.getPovButton(POVDirection.DOWN).whileHeld(ballTransport.runBackwards());
        copilotController.getPovButton(POVDirection.RIGHT).whenPressed(ballTransport.moveBothMotorsToLaser());

        driveController.lbumper().whenPressed(drive.setRotationOffset()).whenReleased(drive.resetRotationOffset());

        // Drive:

        copilotController.b().whenPressed(parallel("Reset Arms", leftClimber.resetArms(), rightClimber.resetArms()));

        driveController.rbumper().whenPressed(drive.setSpeedCoef(0.2)).whenReleased(drive.setSpeedCoef(1.0));

        driveController.y()
                .whileHeld(shootHigh())
                .whenReleased(shooter.stop());
        driveController.x()
                .whileHeld(shootLow())
                .whenReleased(shooter.stop());
        driveController.b()
                .whileHeld(sequence("Shoot", shooter.setTargetVariable(),
                        shooter.waitUntilSpeedUp(),
                        ballTransport.loadShooter(), ballTransport.moveBothMotorsToLaser()))
                .whenReleased(shooter.stop());

        // Intake:

        copilotController.back()
                .whenPressed(parallel("Reset Sequence", leftClimber.resetSequence(), rightClimber.resetSequence()));

        // Both controllers control intake deployment
        driveController.a().or(copilotController.a()).whenActive(intake.extend(
                SpinDirection.COUNTERCLOCKWISE))
                .whileActiveContinuous(ballTransport
                        .loadCargoWithIntake())
                .whenInactive(sequence("Ball transport end",
                        intake.retract(),
                        race("Finish Transport", new WaitCommand(1),
                                ballTransport.loadCargoWithIntake()),
                        ballTransport.stopTransport()));

        // Stop all subsystems
        driveController.back()
                .whenPressed(
                        sequence("Stop All", safeStateSubsystems(ballTransport, drive, intake, shooter),
                                drive.resetCmd()));

        copilotController.lbumper()
                .whileHeld(parallel("Extend Auto",
                        leftClimber.autoClimb(),
                        rightClimber.autoClimb()));

        // Individual control of the elevator arms
        copilotController.rbumper().whileHeld(

                parallel("Climb Individual",

                        leftClimber.climb(
                                deadbandAxis(0.15, () -> -copilotController.getLeftY()),
                                () -> 0),

                        rightClimber.climb(
                                deadbandAxis(0.15, () -> -copilotController.getRightY()),
                                () -> 0)

                )

        );
    }

    @Override
    public void populateDashboard() {
        SmartDashboard.putData("Reset Odometry", new InstantCommand(() -> drive.resetOdometry(new Pose2d()), drive));
        // SmartDashboard.putData("Reset POSE for auto",
        // drive.resetAuto(AutoPaths.twoBallLeftOne));
        SmartDashboard.putData("Update LEDS", led.serialPortSend());

    }

    @Override
    public void setDefaultCommands() {
        final DoubleSupplier deadbandLeftX = deadbandAxis(0.15, driveController::getLeftX);
        final DoubleSupplier deadbandLeftY = deadbandAxis(0.15, driveController::getLeftY);
        final DoubleSupplier deadbandRightX = deadbandAxis(0.15, driveController::getRightX);
        ballTransport.setDefaultCommand(ballTransport.defaultToLaser());
        drive.setDefaultCommand(drive.fieldCentricDrive(deadbandLeftX, deadbandLeftY, deadbandRightX));

        // Manual control for climbing
        leftClimber.setDefaultCommand(leftClimber.climb(
                deadbandAxis(0.15, () -> copilotController.getTriggers()),
                deadbandAxis(0.15, () -> -copilotController.getLeftY())));

        rightClimber.setDefaultCommand(rightClimber.climb(
                deadbandAxis(0.15, () -> copilotController.getTriggers()),
                deadbandAxis(0.15, () -> -copilotController.getLeftY())));

        led.setDefaultCommand(led.animate(teamColors, 1.0));
    }

    public CommandBase safeStateSubsystems(final SmartSubsystem... subsystems) {
        return parallel("Reset Subsystems",
                Stream.of(subsystems).map(SmartSubsystem::safeStateCmd).toArray(CommandBase[]::new));
    }
}