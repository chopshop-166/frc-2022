package frc.robot;

import java.util.function.DoubleSupplier;
import java.util.function.DoubleUnaryOperator;
import java.util.stream.Stream;

import com.chopshop166.chopshoplib.commands.CommandRobot;
import com.chopshop166.chopshoplib.commands.SmartSubsystem;
import com.chopshop166.chopshoplib.controls.ButtonXboxController;
import com.chopshop166.chopshoplib.controls.ButtonXboxController.POVDirection;
import com.chopshop166.chopshoplib.states.SpinDirection;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
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
    public Command getAutoCommand() {
        // Shoot one ball and taxi
        return instant("", () -> {
        });
    }

    @Override
    public void teleopInit() {
        sequence("Init Arms",

                parallel("Zero Arms", leftClimber.resetArms(), rightClimber.resetArms()),

                parallel("Move Past Limits", leftClimber.extendDistanceIgnoreLimit(20),
                        rightClimber.extendDistanceIgnoreLimit(20))

        ).schedule();
    }

    @Override
    public void autonomousInit() {
        leftClimber.resetEncoders();
        rightClimber.resetEncoders();
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

        driveController.start().whenPressed(drive.resetGyro());

        copilotController.getPovButton(POVDirection.UP).whileHeld(ballTransport.runForwards());
        copilotController.getPovButton(POVDirection.DOWN).whileHeld(ballTransport.runBackwards());
        copilotController.getPovButton(POVDirection.RIGHT).whenPressed(ballTransport.moveBothMotorsToLaser());

        driveController.lbumper().whenPressed(drive.setRotationOffset()).whenReleased(drive.resetRotationOffset());

        // Drive:

        driveController.rbumper().whenPressed(drive.setSpeedCoef(0.5)).whenReleased(drive.setSpeedCoef(1.0));

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

        copilotController.rbumper().whileHeld(

                parallel("Climb Individual",

                        leftClimber.climb(
                                deadbandAxis(0.15, () -> -copilotController.getLeftY()),
                                deadbandAxis(0.15, () -> 0)),

                        rightClimber.climb(
                                deadbandAxis(0.15, () -> -copilotController.getRightY()),
                                deadbandAxis(0.15, () -> 0))

                )

        );
    }

    @Override
    public void populateDashboard() {
        SmartDashboard.putData("Run Top Backwards", ballTransport.runTopBackwards());
        SmartDashboard.putData("Run Bottom Backwards", ballTransport.runBottomBackwards());
        SmartDashboard.putData("Only Roll Intake Forwards", intake.startRoller(SpinDirection.COUNTERCLOCKWISE));
        SmartDashboard.putData("Stop Intake", intake.stopRoller());

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