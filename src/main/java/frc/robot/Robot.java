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
import edu.wpi.first.wpilibj2.command.Command;
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

    private UsbCamera camera = CameraServer.startAutomaticCapture();
    private final RobotMap map = getRobotMap(RobotMap.class, "frc.robot.maps", new RobotMap());

    public final Drive drive = new Drive(map.getSwerveDriveMap());
    public final Intake intake = new Intake(map.getIntakeMap());
    public final BallTransport ballTransport = new BallTransport(map.getBallTransportMap());
    public final Shooter shooter = new Shooter(map.getShooterMap());
    public final Climber leftClimber = new Climber(map.getLeftClimberMap(), "Left", ClimberSide.LEFT);
    public final Climber rightClimber = new Climber(map.getRightClimberMap(), "Right", ClimberSide.RIGHT);

    private final Led led = new Led(map.getLedMap());

    private Auto auto = new Auto(drive, intake, ballTransport, shooter, leftClimber, rightClimber);

    private final LightAnimation teamColors = new LightAnimation("rotate.json", "Team Colors");

    @Override
    public void teleopInit() {
        drive.setInverted(false);
        leftClimber.resetEncoders();
        rightClimber.resetEncoders();
        leftClimber.resetSteps();
        rightClimber.resetSteps();
        SmartDashboard.putNumber("High Goal Speed", HubSpeed.LOW.get());
    }

    @Autonomous(defaultAuto = true)
    private CommandBase twoLeftAuto = auto.twoLeftAuto();
    @Autonomous
    private CommandBase weekTwoAutoLow = auto.weekTwoAutoLow();
    @Autonomous
    private CommandBase weekTwoAutoHigh = auto.weekTwoAutoHigh();
    @Autonomous
    private CommandBase onlyShoot = auto.onlyShoot();
    @Autonomous
    private CommandBase delayedAuto = auto.delayedAuto();
    @Autonomous
    private CommandBase offsetTestAuto = auto.offsetAuto();

    private CommandBase shoot(HubSpeed speed, double wait) {
        return sequence("Shoot", shooter.setTargetAndStartShooter(speed),
                shooter.waitUntilSpeedUp(),
                ballTransport.loadShooter(), ballTransport.moveBothMotorsToLaser(), new WaitCommand(wait));
    }

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
                .whileHeld(shoot(HubSpeed.HIGH, 0.7))
                .whenReleased(shooter.stop());
        driveController.x()
                .whileHeld(shoot(HubSpeed.LOW_HIGH_HOOD, 0.0))
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

                ));
    }

    @Override
    public void populateDashboard() {
        SmartDashboard.putData("Reset Odometry", new InstantCommand(() -> drive.resetOdometry(new Pose2d()), drive));
        // SmartDashboard.putData("Reset POSE for auto",
        // drive.resetAuto(AutoPaths.twoBallLeftOne));
        SmartDashboard.putData("Update LEDS", led.serialPortSend());
        SmartDashboard.putNumber("Auto Delay", 7.0);

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