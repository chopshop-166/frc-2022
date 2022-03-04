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
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Shooter.HubSpeed;

public class Robot extends CommandRobot {

    private final ButtonXboxController driveController = new ButtonXboxController(0);
    private final ButtonXboxController copilotController = new ButtonXboxController(1);

    private final RobotMap map = getRobotMap(RobotMap.class, "frc.robot.maps", new RobotMap());

    private final Drive drive = new Drive(map.getSwerveDriveMap());

    private final Intake intake = new Intake(map.getIntakeMap());

    private final BallTransport ballTransport = new BallTransport(map.getBallTransportMap());

    private final Shooter shooter = new Shooter(map.getShooterMap());
    private final Climber leftClimber = new Climber(map.getLeftClimberMap());
    private final Climber rightClimber = new Climber(map.getRightClimberMap());

    @Override
    public void robotInit() {
        super.robotInit();
    }

    @Override
    public void configureButtonBindings() {
        DoubleSupplier climberTrigger = copilotController::getTriggers;
        DoubleSupplier climberJoystickX = copilotController::getLeftX;

        // Shooter:
        // Set target hub for shooter
        driveController.getPovButton(POVDirection.UP).whenPressed(shooter.setTargetHub(HubSpeed.HIGH));
        driveController.getPovButton(POVDirection.DOWN).whenPressed(shooter.setTargetHub(HubSpeed.LOW));

        driveController.b().whileHeld(shooter.testSpeed(0.4));
        driveController.y().whenPressed(shooter.stop());
        // Variable speed for shooter (Used for testing?)
        driveController.lbumper().whileHeld(shooter.setSpeed(copilotController::getLeftTriggerAxis));

        // Drive:
        driveController.back().whenPressed(drive.resetCmd());

        driveController.x().whenPressed(ballTransport.loadShooter());

        SmartDashboard.putData("Run Top Backwards", ballTransport.runTopBackwards());
        SmartDashboard.putData("Run Bottom Backwards", ballTransport.runBottomBackwards());
        SmartDashboard.putData("Only Roll Intake Forwards", intake.startRoller(SpinDirection.COUNTERCLOCKWISE));
        SmartDashboard.putData("Stop Intake", intake.stopRoller());

        // Intake:
        driveController.a().whenPressed(intake.extend(SpinDirection.COUNTERCLOCKWISE))
                .whileHeld(ballTransport.loadCargoWithIntake())
                .whenReleased(sequence("Ball transport end",
                        race("Finish Transport", new WaitCommand(2.5),
                                ballTransport.loadCargoWithIntake()),
                        parallel("Intake retracted w/ Ball Transport",
                                ballTransport.stopTransport(), intake.retract())));
        driveController.y()
                .whenPressed(sequence("Remove Wrong Colored Balls",
                        intake.extend(SpinDirection.COUNTERCLOCKWISE),
                        ballTransport.removeCargo(), intake.retract()));

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

        // Stop all subsystems
        driveController.back().whenPressed(cmd("Stop All").onInitialize(() -> {
            safeStateAll();
        }));
    }

    @Override
    public void populateDashboard() {

    }

    @Override
    public void setDefaultCommands() {
        drive.setDefaultCommand(
                drive.fieldCentricDrive(driveController::getLeftX, driveController::getLeftY,
                        driveController::getRightX));
        ballTransport.setDefaultCommand(ballTransport.defaultToLaser());
    }
}