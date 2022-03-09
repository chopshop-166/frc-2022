package frc.robot;

import java.util.function.DoubleSupplier;
import java.util.stream.Stream;

import com.chopshop166.chopshoplib.commands.CommandRobot;
import com.chopshop166.chopshoplib.commands.SmartSubsystem;
import com.chopshop166.chopshoplib.controls.ButtonXboxController;
import com.chopshop166.chopshoplib.controls.ButtonXboxController.POVDirection;
import com.chopshop166.chopshoplib.states.SpinDirection;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.maps.RobotMap;
import frc.robot.subsystems.BallTransport;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Climber.ExtendDirection;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Led;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Shooter.HubSpeed;
import frc.robot.util.LightAnimation;

public class Robot extends CommandRobot {

    private final ButtonXboxController driveController = new ButtonXboxController(0);
    private final ButtonXboxController copilotController = new ButtonXboxController(1);

    private final RobotMap map = getRobotMap(RobotMap.class, "frc.robot.maps", new RobotMap());

    private final Drive drive = new Drive(map.getSwerveDriveMap());

    private final Intake intake = new Intake(map.getIntakeMap());

    private final BallTransport ballTransport = new BallTransport(map.getBallTransportMap());
    private final Led led = new Led(map.getLedMap());

    private final Shooter shooter = new Shooter(map.getShooterMap());
    private final Climber leftClimber = new Climber(map.getLeftClimberMap());
    private final Climber rightClimber = new Climber(map.getRightClimberMap());

    private final LightAnimation rainbowAnimation = new LightAnimation("rainbow.json", "Rainbow");
    private final LightAnimation redAnimation = new LightAnimation("redfade.json", "Red Fade");
    private final LightAnimation blueAnimation = new LightAnimation("bluefade.json", "Blue Fade");

    @Override
    public void robotInit() {
        super.robotInit();
    }

    @Override
    public void configureButtonBindings() {
        DoubleSupplier climberTrigger = copilotController::getTriggers;
        DoubleSupplier climberJoystickX = copilotController::getLeftX;

        driveController.start().whenPressed(drive.resetGyro());

        copilotController.getPovButton(POVDirection.UP).whileHeld(ballTransport.runForwards());
        copilotController.getPovButton(POVDirection.DOWN).whileHeld(ballTransport.runBackwards());
        copilotController.getPovButton(POVDirection.RIGHT).whenPressed(ballTransport.moveBothMotorsToLaser());

        driveController.getPovButton(POVDirection.UP).whenPressed(drive.driveDistance(1, 0, 0.2));

        // Drive:

        driveController.b().whenPressed(drive.setSpeedCoef(0.5)).whenReleased(drive.setSpeedCoef(1.0));

        driveController.x()
                .whileHeld(sequence("Shoot", shooter.setTargetAndStartShooter(HubSpeed.LOW),
                        shooter.waitUntilSpeedUp(),
                        ballTransport.loadShooter(), ballTransport.moveBothMotorsToLaser()))
                .whenReleased(shooter.stop());

        // Intake:
        // These commands are duplicated for both the drive and copilot controllers.
        copilotController.lbumper().whenPressed(sequence("Remove Wrong Colored Balls",
                intake.extend(SpinDirection.COUNTERCLOCKWISE),
                ballTransport.removeCargo(),
                new WaitCommand(0.5),
                ballTransport.stopTransport(),
                intake.retract()));
        driveController.a().or(copilotController.a()).whenActive(intake.extend(
                SpinDirection.COUNTERCLOCKWISE))
                .whileActiveContinuous(ballTransport
                        .loadCargoWithIntake())
                .whenInactive(sequence("Ball transport end",
                        intake.retract(),
                        race("Finish Transport", new WaitCommand(0.5),
                                ballTransport.loadCargoWithIntake()),
                        ballTransport.stopTransport()));

        driveController.y().or(copilotController.y())
                .whenActive(intake.extend(SpinDirection.CLOCKWISE))
                .whenInactive(intake.retract());
        boolean climberActive = false;
        if (climberActive) { // Climber:
            copilotController.x()
                    .whileHeld(parallel("Extend Triggers", leftClimber.extendSpeed(
                            climberTrigger), rightClimber.extendSpeed(climberTrigger)));
            copilotController.y()
                    .whileHeld(parallel("Rotate", leftClimber.rotateSpeed(
                            climberJoystickX),
                            rightClimber.rotateSpeed(
                                    climberJoystickX)));

            copilotController.getPovButton(POVDirection.LEFT)
                    .whileHeld(parallel("Rotate CCW",
                            leftClimber.rotate(SpinDirection.COUNTERCLOCKWISE),
                            rightClimber.rotate(SpinDirection.COUNTERCLOCKWISE)));
            copilotController.getPovButton(POVDirection.RIGHT)
                    .whileHeld(parallel("Rotate CW", leftClimber.rotate(SpinDirection.CLOCKWISE),
                            rightClimber.rotate(SpinDirection.CLOCKWISE)));
            copilotController.a()
                    .whileHeld(parallel("Extend", leftClimber.extend(ExtendDirection.EXTEND),
                            rightClimber.extend(ExtendDirection.EXTEND)));
            copilotController.b()
                    .whileHeld(parallel("Retract", leftClimber.extend(ExtendDirection.RETRACT),
                            rightClimber.extend(ExtendDirection.RETRACT)));
        }
        // Stop all subsystems
        driveController.back()
                .whenPressed(
                        sequence("Stop All", safeStateSubsystems(ballTransport, drive, intake, shooter),
                                drive.resetCmd()));
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
        drive.setDefaultCommand(
                drive.fieldCentricDrive(driveController::getLeftX, driveController::getLeftY,
                        driveController::getRightX));
        ballTransport.setDefaultCommand(ballTransport.defaultToLaser());
        led.setDefaultCommand(led.animate(rainbowAnimation, 0.1));
    }

    public CommandBase safeStateSubsystems(final SmartSubsystem... subsystems) {
        return parallel("Reset Subsystems",
                Stream.of(subsystems).map(SmartSubsystem::safeStateCmd).toArray(CommandBase[]::new));
    }
}