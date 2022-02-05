package frc.robot;

import com.chopshop166.chopshoplib.commands.CommandRobot;
import com.chopshop166.chopshoplib.controls.ButtonXboxController;
import com.chopshop166.chopshoplib.states.SpinDirection;

import frc.robot.maps.RobotMap;
import frc.robot.subsystems.Intake;

public class Robot extends CommandRobot {

  private final ButtonXboxController controller = new ButtonXboxController(0);

  private final RobotMap map = getMapForName("Gaston", RobotMap.class, "frc.robot.maps");

  private final Intake intake = new Intake(map.getIntakeMap());

  @Override
  public void robotInit() {
    super.robotInit();
  }

  @Override
  public void configureButtonBindings() {
    controller.a().whileHeld(intake.startIntakeMechanism(SpinDirection.COUNTERCLOCKWISE));
  }

  @Override
  public void populateDashboard() {

  }

  @Override
  public void setDefaultCommands() {

  }
}
