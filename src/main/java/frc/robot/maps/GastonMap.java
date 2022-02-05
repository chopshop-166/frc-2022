package frc.robot.maps;

import com.chopshop166.chopshoplib.drive.SDSSwerveModule;
import com.chopshop166.chopshoplib.motors.PIDSparkMax;
import com.chopshop166.chopshoplib.sensors.PigeonGyro;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.DigitalInput;


public class GastonMap extends RobotMap {
    @Override
    public SwerveDriveMap getSwerveDriveMap() {
        // Value taken from CAD as offset from center of module base pulley to center of
        // robot
        final double MODULE_OFFSET_XY = 0.314325; // TODO Get this value

        // All Distances are in Meters
        // Front Left Module
        final CANCoder encoderFL = new CANCoder(1);
        encoderFL.configMagnetOffset(0); // TODO Get Magnet Offset
        encoderFL.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
        final SDSSwerveModule frontLeft = new SDSSwerveModule(
                new Translation2d(MODULE_OFFSET_XY, MODULE_OFFSET_XY),
                encoderFL,
                new PIDSparkMax(2, MotorType.kBrushless), new PIDSparkMax(1, MotorType.kBrushless),
                SDSSwerveModule.MK4_V2);

        // Front Right Module
        final CANCoder encoderFR = new CANCoder(2);
        encoderFR.configMagnetOffset(0); // TODO Get Magnet Offset
        encoderFR.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
        final SDSSwerveModule frontRight = new SDSSwerveModule(
                new Translation2d(MODULE_OFFSET_XY, -MODULE_OFFSET_XY),
                encoderFR,
                new PIDSparkMax(4, MotorType.kBrushless), new PIDSparkMax(3, MotorType.kBrushless),
                SDSSwerveModule.MK4_V2);

        // Rear Left Module
        final CANCoder encoderRL = new CANCoder(3);
        encoderRL.configMagnetOffset(0); // TODO Get Magnet Offset
        encoderRL.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
        final SDSSwerveModule rearLeft = new SDSSwerveModule(
                new Translation2d(-MODULE_OFFSET_XY, MODULE_OFFSET_XY),
                encoderRL,
                new PIDSparkMax(6, MotorType.kBrushless), new PIDSparkMax(5, MotorType.kBrushless),
                SDSSwerveModule.MK4_V2);

        // Rear Right Module
        final CANCoder encoderRR = new CANCoder(4);
        encoderRR.configMagnetOffset(0); // TODO Get Magnet Offset
        encoderRR.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
        final SDSSwerveModule rearRight = new SDSSwerveModule(
                new Translation2d(-MODULE_OFFSET_XY, -MODULE_OFFSET_XY),
                encoderRR,
                new PIDSparkMax(8, MotorType.kBrushless), new PIDSparkMax(7, MotorType.kBrushless),
                SDSSwerveModule.MK4_V2);

        final double maxDriveSpeedMetersPerSecond = Units.feetToMeters(10);

        final double maxRotationRadianPerSecond = Math.PI;

        final Gyro gyro = new PigeonGyro(new PigeonIMU(5));

        return new SwerveDriveMap(frontLeft, frontRight, rearLeft, rearRight, maxDriveSpeedMetersPerSecond,
                        maxRotationRadianPerSecond, gyro);
    }
    private final DigitalInput leftUpperLimit = new DigitalInput(0);
    private final DigitalInput leftLowerLimit = new DigitalInput(1);
    private final DigitalInput rightUpperLimit = new DigitalInput(2);
    private final DigitalInput rightLowerLimit = new DigitalInput(3);

    private final PIDSparkMax leftMotor = new PIDSparkMax(9, MotorType.kBrushless);
    private final PIDSparkMax rightMotor = new PIDSparkMax(10, MotorType.kBrushless);

    @Override
    public TelescopeMap getLeftTelescopeMap() {
        return new TelescopeMap(leftMotor, leftUpperLimit::get, leftLowerLimit::get);
    }
    @Override
    public TelescopeMap getRightTelescopeMap() {
        return new TelescopeMap(rightMotor, rightUpperLimit::get, rightLowerLimit::get);
    }
}
