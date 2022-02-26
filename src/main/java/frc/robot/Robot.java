package frc.robot;

import java.util.function.DoubleSupplier;

import com.chopshop166.chopshoplib.commands.CommandRobot;
import com.chopshop166.chopshoplib.controls.ButtonXboxController;
import com.chopshop166.chopshoplib.controls.ButtonXboxController.POVDirection;
import com.chopshop166.chopshoplib.states.SpinDirection;

import frc.robot.maps.RobotMap;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Climber.ExtendDirection;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Intake;

public class Robot extends CommandRobot {

  private final ButtonXboxController driveController = new ButtonXboxController(0);
  private final ButtonXboxController copilotController = new ButtonXboxController(1);

  private final RobotMap map = getRobotMap(RobotMap.class, "frc.robot.maps", new RobotMap());

  private final Drive drive = new Drive(map.getSwerveDriveMap());

  private final Intake intake = new Intake(map.getIntakeMap());

  private final Climber leftClimber = new Climber(map.getLeftClimberMap());
  private final Climber rightClimber = new Climber(map.getRightClimberMap());

  @Override
  public void robotInit() {
    super.robotInit();
  }

  @Override
  public void configureButtonBindings() {
    DoubleSupplier trigger = copilotController::getTriggers;

    driveController.back().whenPressed(drive.resetCmd());

    // Intake:
    // On button press: extend intake and start roller
    // On button release: retract intake and stop roller
    copilotController.a().whenPressed(intake.extend(SpinDirection.COUNTERCLOCKWISE))
        .whenReleased(intake.retract(SpinDirection.COUNTERCLOCKWISE));

    // Climber:
    copilotController.x()
        .whileHeld(parallel("Extend Triggers", leftClimber.extendSpeed(trigger), rightClimber.extendSpeed(trigger)));
    copilotController.y()
        .whileHeld(parallel("Rotate", leftClimber.rotateSpeed(trigger), rightClimber.rotateSpeed(trigger)));

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
        drive.fieldCentricDrive(driveController::getLeftX, driveController::getLeftY, driveController::getRightX));

  }
}
