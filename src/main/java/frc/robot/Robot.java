package frc.robot;

import java.util.function.DoubleSupplier;

import com.chopshop166.chopshoplib.commands.CommandRobot;
import com.chopshop166.chopshoplib.controls.ButtonXboxController;
import com.chopshop166.chopshoplib.controls.ButtonXboxController.POVDirection;
import com.chopshop166.chopshoplib.states.SpinDirection;

import frc.robot.maps.RobotMap;
import frc.robot.subsystems.Climber;
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
    DoubleSupplier trigger = driveController::getTriggers;

    driveController.back().whenPressed(drive.resetCmd());

    copilotController.a().whileHeld(intake.extend(SpinDirection.COUNTERCLOCKWISE));
    copilotController.b().whileHeld(intake.retract(SpinDirection.COUNTERCLOCKWISE));

    // Move with variable speed from triggers
    driveController.x().whileHeld(parallel("Move", leftClimber.move(trigger), rightClimber.move(trigger)));

    // Button bindings for regular climbing
    driveController.a().whileHeld(parallel("Extend", leftClimber.extend(), rightClimber.extend()));
    driveController.b().whileHeld(parallel("Retract", leftClimber.retract(), rightClimber.retract()));

    // Button bindings for ignoring limit switches
    driveController.getPovButton(POVDirection.UP)
        .whileHeld(parallel("Extend Ignore Limit", leftClimber.extendIgnoreLimit(), rightClimber.extendIgnoreLimit()));
    driveController.getPovButton(POVDirection.DOWN).whileHeld(
        parallel("Retract Ignore Limit", leftClimber.retractIgnoreLimit(), rightClimber.retractIgnoreLimit()));

    driveController.start().whenPressed(parallel("Stop", leftClimber.stop(), rightClimber.stop()));
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
