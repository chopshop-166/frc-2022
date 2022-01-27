package frc.robot;

import com.chopshop166.chopshoplib.commands.CommandRobot;
import com.chopshop166.chopshoplib.controls.ButtonXboxController;

import frc.robot.maps.RobotMap;
import frc.robot.subsystems.Drive;

public class Robot extends CommandRobot {

  final private ButtonXboxController driveController = new ButtonXboxController(0);
  final private ButtonXboxController copilotController = new ButtonXboxController(1);

  final private RobotMap map = getRobotMap(RobotMap.class, "frc.robot.maps", new RobotMap());

  private final Drive drive = new Drive(map.getDriveMap());

  @Override
  public void robotInit() {
    // Nothing to initialize
  }

  @Override
  public void configureButtonBindings() {
    driveController.start().whenPressed(drive.resetCmd());
  }

  @Override
  public void populateDashboard() {
    // TODO Auto-generated method stub

  }

  @Override
  public void setDefaultCommands() {
    // drive.setDefaultCommand(drive.fieldCentricDrive(() ->
    // driveController.getX(Hand.kLeft),
    // () -> driveController.getY(Hand.kLeft), () ->
    // driveController.getX(Hand.kRight)));

  }
}
