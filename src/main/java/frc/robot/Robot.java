package frc.robot;

import com.chopshop166.chopshoplib.commands.CommandRobot;

import frc.robot.maps.RobotMap;
import frc.robot.maps.RobotMap.ShooterMap;
import frc.robot.subsystems.Shooter;

public class Robot extends CommandRobot {

  private RobotMap map = getMapForName("Gaston", RobotMap.class, "frc.robot.maps"); // ?gets map from Gaston.java
  private Shooter shooter = new Shooter(map.getShooterMap()); // ?gets shootermap and makes a shooter subsystem

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
