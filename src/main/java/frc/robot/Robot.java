package frc.robot;

import com.chopshop166.chopshoplib.commands.CommandRobot;

import frc.robot.maps.RobotMap;
import frc.robot.subsystems.Drive;

public class Robot extends CommandRobot {

  final private RobotMap map = getRobotMap(RobotMap.class, "frc.robot.maps", new RobotMap());

  private final Drive drive = new Drive(map.getDriveMap());

  @Override
  public void robotInit() {
    // Nothing to initialize
  }

  @Override
  public void configureButtonBindings() {
    // TODO Auto-generated method stub

  }

  @Override
  public void populateDashboard() {
    // TODO Auto-generated method stub

  }

  @Override
  public void setDefaultCommands() {
    // TODO Auto-generated method stub

  }
}
