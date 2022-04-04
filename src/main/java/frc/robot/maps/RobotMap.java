package frc.robot.maps;

import com.chopshop166.chopshoplib.drive.SwerveDriveMap;
import com.chopshop166.chopshoplib.maps.RobotMapFor;

import frc.robot.maps.subsystems.BallTransportMap;
import frc.robot.maps.subsystems.ClimberMap;
import frc.robot.maps.subsystems.IntakeMap;
import frc.robot.maps.subsystems.LedMap;
import frc.robot.maps.subsystems.ShooterMap;

@RobotMapFor("Default")
public class RobotMap {

    public SwerveDriveMap getSwerveDriveMap() {
        return new SwerveDriveMap();
    }

    public BallTransportMap getBallTransportMap() {
        return new BallTransportMap();
    }

    public IntakeMap getIntakeMap() {
        return new IntakeMap();
    }

    public ShooterMap getShooterMap() {
        return new ShooterMap();
    }

    public ClimberMap getLeftClimberMap() {
        return new ClimberMap();
    }

    public ClimberMap getRightClimberMap() {
        return new ClimberMap();
    }

    public LedMap getLedMap() {
        return new LedMap();
    }

    public LedMap getLeftLedMap() {
        return new LedMap();
    }

    public LedMap getRightLedMap() {
        return new LedMap();
    }
}
