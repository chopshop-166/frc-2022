package frc.robot;

import com.chopshop166.chopshoplib.commands.CommandRobot;
import com.chopshop166.chopshoplib.controls.ButtonXboxController;

import frc.robot.maps.RobotMap;
import frc.robot.subsystems.Climber;

public class Robot extends CommandRobot {

  private final ButtonXboxController controller = new ButtonXboxController(0);

  private final RobotMap map = getMapForName("Gaston", RobotMap.class, "frc.robot.maps");

  private final Climber leftClimber = new Climber(map.getLeftTelescopeMap());
  private final Climber rightClimber = new Climber(map.getRightTelescopeMap());

  @Override
  public void robotInit() {
    super.robotInit();
  }

  @Override
  public void configureButtonBindings() {

    // Button bindings for regular climbing
    controller.a().whileHeld(parallel("Extend", leftClimber.extend(), rightClimber.extend()));
    controller.b().whileHeld(parallel("Retract", leftClimber.retract(), rightClimber.retract()));

    // Button bindings for ignoring limit switches
    controller.x()
        .whileHeld(parallel("Extend Ignore Limit", leftClimber.extendIgnoreLimit(), rightClimber.extendIgnoreLimit()));
    controller.y().whileHeld(
        parallel("Retract Ignore Limit", leftClimber.retractIgnoreLimit(), rightClimber.retractIgnoreLimit()));

    controller.start().whenPressed(parallel("Stop", leftClimber.stop(), rightClimber.stop()));

  }

  @Override
  public void populateDashboard() {
    // TODO Auto-generated method stub

  }

  @Override
  public void setDefaultCommands() {

  }
}
