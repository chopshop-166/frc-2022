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
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.maps.RobotMap;
import frc.robot.subsystems.BallTransport;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Led;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Shooter.HubSpeed;
import frc.robot.util.LightAnimation;

public class Robot extends CommandRobot {

    private final ButtonXboxController driveController = new ButtonXboxController(0);
    private final ButtonXboxController copilotController = new ButtonXboxController(1);

    private UsbCamera camera = CameraServer.startAutomaticCapture();
    private final RobotMap map = getRobotMap(RobotMap.class, "frc.robot.maps", new RobotMap());

    private final Drive drive = new Drive(map.getSwerveDriveMap());

    private final Intake intake = new Intake(map.getIntakeMap());

    private final BallTransport ballTransport = new BallTransport(map.getBallTransportMap());
    private final Led led = new Led(map.getLedMap());

    private final Shooter shooter = new Shooter(map.getShooterMap());
    private final Climber leftClimber = new Climber(map.getLeftClimberMap(), "Left");
    private final Climber rightClimber = new Climber(map.getRightClimberMap(), "Right");

    private final LightAnimation teamColors = new LightAnimation("rotate.json", "Team Colors");

    @Override
    public void teleopInit() {
        // sequence("Init Arms",

        // parallel("Zero Arms", leftClimber.resetArms(), rightClimber.resetArms())

        // ).schedule();

        leftClimber.resetEncoders();
        rightClimber.resetEncoders();
        leftClimber.resetSteps();
        rightClimber.resetSteps();
    }

    public CommandBase shootOneBallAuto() {
        return sequence("Shoot Preloaded Ball", shooter.setTargetAndStartShooter(HubSpeed.LOW),
                shooter.waitUntilSpeedUp(), ballTransport.loadShooter());
    }

    public CommandBase shootTwoBallsAuto() {
        return sequence("Shoot Two Balls Auto", shooter.setTargetAndStartShooter(HubSpeed.LOW),
                shooter.waitUntilSpeedUp(), ballTransport.loadShooter(), ballTransport.moveBothMotorsToLaser(),
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
    public CommandBase threeRightAuto() {
        return sequence("Three Ball Right Auto",
                shootOneBallAuto(),
                parallel("Stop Shooter", stopShooter(),
                        drive.auto(AutoPaths.threeBallRightOne)),
                intakeOneBallAuto(), drive.auto(AutoPaths.threeBallRightTwo),
                intakeOneBallAuto(), drive.auto(AutoPaths.threeBallRightThree), shootTwoBallsAuto(), stopShooter());
    }

    public CommandBase twoRightAuto() {
        return sequence("Two Ball Right Auto", shootOneBallAuto(),
                parallel("Stop Shooter", stopShooter(), drive.auto(
                        AutoPaths.twoBallRightOne)),
                intakeOneBallAuto(),
                drive.auto(
                        AutoPaths.twoBallRightTwo),
                shootOneBallAuto(),
                parallel("Stop Shooter", stopShooter(), drive.auto(AutoPaths.twoBallRightOne)));
    }

    public CommandBase twoLeftAuto() {
        return sequence("Two Ball Left Auto", drive.resetAuto(AutoPaths.twoBallLeftOne),
                parallel("Stop Shooter", stopShooter(), drive.auto(
                        AutoPaths.twoBallLeftOne)),
                intakeOneBallAuto(), drive.auto(AutoPaths.twoBallLeftTwo),
                shootOneBallAuto(), parallel("Stop Shooter", stopShooter()));
    }

    // Shoot One ball and taxi
    public CommandBase oneBallAuto() {
        return sequence("One Ball Auto", shootOneBallAuto(),
                drive.auto(AutoPaths.oneBallLeftOne));
    }

    private CommandBase weekTwoAuto() {
        return sequence("Autonomous",
                shooter.setTargetAndStartShooter(HubSpeed.LOW),
                shooter.waitUntilSpeedUp(),
                ballTransport.loadShooter(), ballTransport.moveBothMotorsToLaser(),

                parallel("Stop and drive",
                        sequence("Stop shooter", new WaitCommand(2), shooter.stop()),
                        drive.driveDistance(2.5, 0, 0.5)));
    }

    @Autonomous
    public CommandBase threeRightAuto = threeRightAuto();
    @Autonomous
    public CommandBase twoRightAuto = twoRightAuto();
    @Autonomous
    public CommandBase twoLeftAuto = twoLeftAuto();
    @Autonomous(defaultAuto = true)
    public CommandBase oneBallAuto = oneBallAuto();
    @Autonomous
    public CommandBase weekTwoAuto = weekTwoAuto();

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
        Shuffleboard.getTab("Camera").add("USB Camera 0", camera);

    }

    @Override
    public void configureButtonBindings() {

        driveController.start().whenPressed(drive.resetGyro());

        copilotController.getPovButton(POVDirection.UP).whileHeld(ballTransport.runForwards());
        copilotController.getPovButton(POVDirection.DOWN).whileHeld(ballTransport.runBackwards());
        copilotController.getPovButton(POVDirection.RIGHT).whenPressed(ballTransport.moveBothMotorsToLaser());

        driveController.lbumper().whenPressed(drive.setRotationOffset()).whenReleased(drive.resetRotationOffset());

        // Drive:

        driveController.rbumper().whenPressed(drive.setSpeedCoef(0.2)).whenReleased(drive.setSpeedCoef(1.0));

        driveController.x()
                .whileHeld(sequence("Shoot", shooter.setTargetAndStartShooter(HubSpeed.LOW),
                        shooter.waitUntilSpeedUp(),
                        ballTransport.loadShooter(), ballTransport.moveBothMotorsToLaser()))
                .whenReleased(shooter.stop());

        // Intake:
        // These commands are duplicated for both the drive and copilot controllers.

        driveController.a().or(copilotController.a()).whenActive(intake.extend(
                SpinDirection.COUNTERCLOCKWISE))
                .whileActiveContinuous(ballTransport
                        .loadCargoWithIntake())
                .whenInactive(sequence("Ball transport end",
                        intake.retract(),
                        race("Finish Transport", new WaitCommand(1),
                                ballTransport.loadCargoWithIntake()),
                        ballTransport.stopTransport()));

        driveController.y().or(copilotController.y())
                .whenActive(intake.extend(SpinDirection.CLOCKWISE))
                .whenInactive(intake.retract());

        // Stop all subsystems
        driveController.back()
                .whenPressed(
                        sequence("Stop All", safeStateSubsystems(ballTransport, drive, intake, shooter),
                                drive.resetCmd()));

        copilotController.lbumper()
                .whileHeld(parallel("Extend Auto",
                        leftClimber.autoClimb(),
                        rightClimber.autoClimb()));

        copilotController.b().whenPressed(parallel("Reset Arms", leftClimber.resetArms(), rightClimber.resetArms()));

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
        SmartDashboard.putData("Reset POSE for auto", drive.resetAuto(AutoPaths.twoBallLeftOne));

    }

    @Override
    public void setDefaultCommands() {
        final DoubleSupplier deadbandLeftX = deadbandAxis(0.15, driveController::getLeftX);
        final DoubleSupplier deadbandLeftY = deadbandAxis(0.15, driveController::getLeftY);
        final DoubleSupplier deadbandRightX = deadbandAxis(0.15, driveController::getRightX);
        ballTransport.setDefaultCommand(ballTransport.defaultToLaser());
        drive.setDefaultCommand(drive.fieldCentricDrive(deadbandLeftX, deadbandLeftY, deadbandRightX));
        // Eventually use controls for rotating arms
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