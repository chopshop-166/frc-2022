package frc.robot;

import com.chopshop166.chopshoplib.commands.CommandRobot;
import com.chopshop166.chopshoplib.controls.ButtonXboxController;

import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.maps.RobotMap;
import frc.robot.subsystems.Shooter;

public class Robot extends CommandRobot {

  private RobotMap map = getMapForName("Gaston", RobotMap.class, "frc.robot.maps"); // ?gets map from Gaston.java
  private Shooter shooter = new Shooter(map.getShooterMap()); // ?gets shootermap and makes a shooter subsystem

  private final ButtonXboxController controller = new ButtonXboxController(0);

  @Override
  public void robotInit() {
    // Nothing to initialize

  }

  @Override
  public void configureButtonBindings() {
    controller.lbumper().whileHeld(shooter.setSpeed(controller::getLeftTriggerAxis));
    // * when left bumper is pressed lets pilot set speed :)
    controller.rbumper().whenPressed(sequence("shoot",
        new WaitCommand(shooter.getWaitTime()), // * gives the PID controll time to do its thing
        shooter.new shoot()));
    // * get ready, aim, FIRE!!
  }

  @Override
  public void populateDashboard() {
    // TODO Auto-generated method stub

  }

  @Override
  public void setDefaultCommands() {

  }
}