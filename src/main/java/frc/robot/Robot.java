package frc.robot;

import com.chopshop166.chopshoplib.commands.CommandRobot;
import com.chopshop166.chopshoplib.controls.ButtonXboxController;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.maps.RobotMap;
import frc.robot.subsystems.Climber;

public class Robot extends CommandRobot {

  private Command autonomousCommand;

  private final SendableChooser<Command> autoChooser = new SendableChooser<>();

  private final ButtonXboxController controller = new ButtonXboxController(0);

  private final Climber leftClimber = new Climber(RobotMap.getLeftTelescope());
  private final Climber rightClimber = new Climber(RobotMap.getRightTelescope());

  @Override
  public void robotInit() {

  }

  @Override
  public void configureButtonBindings() {
    controller.a().whileHeld(parallel("Extend", leftClimber.extend(), rightClimber.extend()));
  }

  @Override
  public void populateDashboard() {
    // TODO Auto-generated method stub

  }

  @Override
  public void setDefaultCommands() {
    leftClimber.setDefaultCommand(leftClimber.retract());
    rightClimber.setDefaultCommand(rightClimber.retract());
  }
}
