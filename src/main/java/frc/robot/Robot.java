package frc.robot;

import java.util.function.DoubleSupplier;

import com.chopshop166.chopshoplib.commands.CommandRobot;
import com.chopshop166.chopshoplib.controls.ButtonXboxController;
import com.chopshop166.chopshoplib.states.SpinDirection;
import com.chopshop166.chopshoplib.controls.ButtonXboxController.POVDirection;

import frc.robot.maps.RobotMap;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Intake;

public class Robot extends CommandRobot {

  private final ButtonXboxController controller = new ButtonXboxController(0);

  private final RobotMap map = getMapForName("Gaston", RobotMap.class, "frc.robot.maps");

  private final Intake intake = new Intake(map.getIntakeMap());

  private final Climber leftClimber = new Climber(map.getLeftTelescopeMap());
  private final Climber rightClimber = new Climber(map.getRightTelescopeMap());

  @Override
  public void robotInit() {
    super.robotInit();
  }

  @Override
  public void configureButtonBindings() {
    controller.a().whileHeld(intake.startIntakeMechanism(SpinDirection.COUNTERCLOCKWISE));
    DoubleSupplier trigger = controller::getTriggers;

    // Move with variable speed from triggers
    controller.x().whileHeld(parallel("Move", leftClimber.move(trigger), rightClimber.move(trigger)));

    // Button bindings for regular climbing
    controller.a().whileHeld(parallel("Extend", leftClimber.extend(), rightClimber.extend()));
    controller.b().whileHeld(parallel("Retract", leftClimber.retract(), rightClimber.retract()));

    // Button bindings for ignoring limit switches
    controller.getPovButton(POVDirection.UP)
        .whileHeld(parallel("Extend Ignore Limit", leftClimber.extendIgnoreLimit(), rightClimber.extendIgnoreLimit()));
    controller.getPovButton(POVDirection.DOWN).whileHeld(
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
