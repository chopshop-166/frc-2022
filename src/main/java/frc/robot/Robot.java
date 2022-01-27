package frc.robot;

import java.util.function.DoubleSupplier;

import com.chopshop166.chopshoplib.commands.CommandRobot;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.maps.RobotMap;
import frc.robot.maps.RobotMap.ShooterMap;
import frc.robot.subsystems.Shooter;

public class Robot extends CommandRobot {

  private RobotMap map = getMapForName("Gaston", RobotMap.class, "frc.robot.maps"); // ?gets map from Gaston.java
  private Shooter shooter = new Shooter(map.getShooterMap()); // ?gets shootermap and makes a shooter subsystem

  private final XboxController controller = new XboxController(0);

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

    DoubleSupplier shooterSpeed = () -> controller.getLeftTriggerAxis();
    shooter.setDefaultCommand(shooter.setSpeed(shooterSpeed));
  }
}
