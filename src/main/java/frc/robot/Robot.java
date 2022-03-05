package frc.robot;

import java.util.function.DoubleSupplier;

import com.chopshop166.chopshoplib.commands.CommandRobot;
import com.chopshop166.chopshoplib.controls.ButtonXboxController;
import com.chopshop166.chopshoplib.controls.ButtonXboxController.POVDirection;
import com.chopshop166.chopshoplib.states.SpinDirection;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.maps.RobotMap;
import frc.robot.subsystems.BallTransport;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Climber.ExtendDirection;
import frc.robot.util.LightAnimation;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Led;

public class Robot extends CommandRobot {

    private final ButtonXboxController driveController = new ButtonXboxController(0);
    private final ButtonXboxController copilotController = new ButtonXboxController(1);

    private final RobotMap map = getRobotMap(RobotMap.class, "frc.robot.maps", new RobotMap());

    private final Drive drive = new Drive(map.getSwerveDriveMap());

    private final Intake intake = new Intake(map.getIntakeMap());

    private final Led led = new Led(map.getLedMap());

    private final BallTransport ballTransport = new BallTransport(map.getBallTransportMap());

    private final Climber leftClimber = new Climber(map.getLeftClimberMap());
    private final Climber rightClimber = new Climber(map.getRightClimberMap());

    private final LightAnimation rainbowAnimation = new LightAnimation("rainbow.json");

    @Override
    public void robotInit() {
        super.robotInit();
    }

    @Override
    public void configureButtonBindings() {
        DoubleSupplier climberTrigger = copilotController::getTriggers;
        DoubleSupplier climberJoystickX = copilotController::getLeftX;

        driveController.back().whenPressed(drive.resetCmd());

        // Intake:
        // On button press: extend intake and start roller
        // On button release: retract intake and stop roller

        // Soon to be shooter command
        driveController.x().whenPressed(ballTransport.loadShooter());

        driveController.a().whenPressed(intake.extend())
                .whileHeld(
                        sequence("Start Intake and Transporter", intake.startRoller(SpinDirection.COUNTERCLOCKWISE),
                                ballTransport.loadCargoWithIntake()))
                .whenReleased(sequence("Ball transport end", race("Finish Transport", new WaitCommand(2), ballTransport
                        .loadCargoWithIntake()),
                        parallel("Intake retracted w/ Ball Transport", ballTransport.stopTransport(),
                                intake.retract())));
        driveController.y()
                .whenPressed(
                        sequence("Remove Wrong Colored Balls", intake.extend(),
                                intake.startRoller(SpinDirection.CLOCKWISE),
                                ballTransport.removeCargo(), intake.retract()));

        SmartDashboard.putData("Run Top Backwards", ballTransport.runTopBackwards());
        SmartDashboard.putData("Run Bottom Backwards", ballTransport.runBottomBackwards());
        SmartDashboard.putData("Only Roll Intake Forwards", intake.startRoller(SpinDirection.COUNTERCLOCKWISE));
        SmartDashboard.putData("Stop Intake", intake.stopRoller());

        // Climber:
        copilotController.x()
                .whileHeld(parallel("Extend Triggers", leftClimber.extendSpeed(
                        climberTrigger), rightClimber.extendSpeed(climberTrigger)));
        copilotController.y()
                .whileHeld(parallel("Rotate", leftClimber.rotateSpeed(
                        climberJoystickX),
                        rightClimber.rotateSpeed(
                                climberJoystickX)));

        copilotController.getPovButton(POVDirection.LEFT)
                .whileHeld(parallel("Rotate CCW", leftClimber.rotate(SpinDirection.COUNTERCLOCKWISE),
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

        // Stop all subsystems
        copilotController.back().whenPressed(cmd("Stop All").onInitialize(() -> {
            safeStateAll();
        }));
    }

    @Override
    public void populateDashboard() {

        // TODO Auto-generated method stub
    }

    @Override
    public void setDefaultCommands() {
        drive.setDefaultCommand(
                drive.fieldCentricDrive(driveController::getLeftX, driveController::getLeftY,
                        driveController::getRightX));
        ballTransport.setDefaultCommand(ballTransport.defaultToLaser());

        led.setDefaultCommand(led.animate(rainbowAnimation));
    }
}
