package frc.robot;

import com.chopshop166.chopshoplib.commands.CommandRobot;
import com.chopshop166.chopshoplib.controls.ButtonXboxController;

import frc.robot.maps.RobotMap;
import frc.robot.subsystems.Drive;

public class Robot extends CommandRobot {

  private final ButtonXboxController driveController = new ButtonXboxController(0);
  private final ButtonXboxController copilotController = new ButtonXboxController(1);

  private final RobotMap map = getRobotMap(RobotMap.class, "frc.robot.maps", new RobotMap());

  private final Drive drive = new Drive(map.getSwerveDriveMap());

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
    drive.setDefaultCommand(drive.fieldCentricDrive(driveController::getLeftX,
        driveController::getLeftY, driveController::getRightX));
  }
}
