package frc.robot;

import java.util.function.DoubleSupplier;

import com.chopshop166.chopshoplib.commands.CommandRobot;
import com.chopshop166.chopshoplib.controls.ButtonXboxController;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import frc.robot.maps.RobotMap;
import frc.robot.maps.RobotMap.ShooterMap;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Shooter.CheckShootSpeed;

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
    controller.b().whenPressed(sequence("shoot", shooter.new CheckShootSpeed(), shooter.new shoot())); // * get ready,
                                                                                                       // * aim, FIRE
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
