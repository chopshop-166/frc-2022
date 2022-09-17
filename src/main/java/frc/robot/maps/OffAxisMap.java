package frc.robot.maps;

import com.chopshop166.chopshoplib.drive.SDSSwerveModule;
import com.chopshop166.chopshoplib.drive.SwerveDriveMap;
import com.chopshop166.chopshoplib.maps.RobotMapFor;
import com.chopshop166.chopshoplib.motors.PIDSparkMax;
import com.chopshop166.chopshoplib.pneumatics.CtreDSolenoid;
import com.chopshop166.chopshoplib.sensors.gyro.PigeonGyro;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

@RobotMapFor("00:80:2F:19:7B:A3")
public class OffAxisMap extends RobotMap {

    final PigeonGyro pigeonGyro = new PigeonGyro(new PigeonIMU(5));

    // creating the intake object allows WPILIB to control the compressor
    final CtreDSolenoid intake = new CtreDSolenoid(6, 7);

    @Override
    public SwerveDriveMap getSwerveDriveMap() {
        // Value taken from CAD as offset from center of module base pulley to center
        // of the robot
        final double MODULE_OFFSET_XY = 0.381;

        final PIDSparkMax frontLeftSteer = new PIDSparkMax(2, MotorType.kBrushless);
        final PIDSparkMax frontRightSteer = new PIDSparkMax(4, MotorType.kBrushless);
        final PIDSparkMax rearLeftSteer = new PIDSparkMax(6, MotorType.kBrushless);
        final PIDSparkMax rearRightSteer = new PIDSparkMax(8, MotorType.kBrushless);

        frontLeftSteer.getMotorController().setPeriodicFramePeriod(PeriodicFrame.kStatus3, 1000);
        frontRightSteer.getMotorController().setPeriodicFramePeriod(PeriodicFrame.kStatus3, 1000);
        rearLeftSteer.getMotorController().setPeriodicFramePeriod(PeriodicFrame.kStatus3, 1000);
        frontRightSteer.getMotorController().setPeriodicFramePeriod(PeriodicFrame.kStatus3, 1000);

        // All Distances are in Meters
        // Front Left Module
        final CANCoder encoderFL = new CANCoder(1);
        encoderFL.configMagnetOffset(-36.0078125);
        encoderFL.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
        final SDSSwerveModule frontLeft = new SDSSwerveModule(new Translation2d(MODULE_OFFSET_XY, MODULE_OFFSET_XY),
                encoderFL, frontLeftSteer, new PIDSparkMax(1,
                        MotorType.kBrushless),
                SDSSwerveModule.MK3_FAST);

        // Front Right Module
        final CANCoder encoderFR = new CANCoder(2);
        encoderFR.configMagnetOffset(-293.02734375000006);
        encoderFR.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
        final SDSSwerveModule frontRight = new SDSSwerveModule(new Translation2d(MODULE_OFFSET_XY, -MODULE_OFFSET_XY),
                encoderFR, frontRightSteer, new PIDSparkMax(3,
                        MotorType.kBrushless),
                SDSSwerveModule.MK3_FAST);

        // Rear Left Module
        final CANCoder encoderRL = new CANCoder(3);
        encoderRL.configMagnetOffset(-102.6562);
        encoderRL.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
        final SDSSwerveModule rearLeft = new SDSSwerveModule(new Translation2d(-MODULE_OFFSET_XY, MODULE_OFFSET_XY),
                encoderRL, rearLeftSteer, new PIDSparkMax(5,
                        MotorType.kBrushless),
                SDSSwerveModule.MK3_FAST);

        // Rear Right Module
        final CANCoder encoderRR = new CANCoder(4);
        encoderRR.configMagnetOffset(-269.121);
        encoderRR.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
        final SDSSwerveModule rearRight = new SDSSwerveModule(new Translation2d(-MODULE_OFFSET_XY, -MODULE_OFFSET_XY),
                encoderRR, rearRightSteer, new PIDSparkMax(7,
                        MotorType.kBrushless),
                SDSSwerveModule.MK3_FAST);

        final double maxDriveSpeedMetersPerSecond = Units.feetToMeters(10);

        final double maxRotationRadianPerSecond = Math.PI;
        return new SwerveDriveMap(frontLeft, frontRight, rearLeft, rearRight,
                maxDriveSpeedMetersPerSecond,
                maxRotationRadianPerSecond, pigeonGyro);
    }
}
