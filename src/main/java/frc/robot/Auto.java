package frc.robot;

import com.chopshop166.chopshoplib.commands.Commandable;
import com.chopshop166.chopshoplib.states.SpinDirection;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.BallTransport;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Shooter.HubSpeed;

public class Auto implements Commandable {
    Drive drive;
    Intake intake;
    BallTransport ballTransport;
    Shooter shooter;
    Climber leftClimber;
    Climber rightClimber;

    Auto(Drive drive, Intake intake, BallTransport ballTransport, Shooter shooter, Climber leftClimber,
            Climber rightClimber) {
        this.drive = drive;
        this.intake = intake;
        this.ballTransport = ballTransport;
        this.shooter = shooter;
        this.leftClimber = leftClimber;
        this.rightClimber = rightClimber;
    }

    private CommandBase shootHigh() {
        return sequence("Shoot High", shooter.setTargetAndStartShooter(HubSpeed.HIGH),
                shooter.waitUntilSpeedUp(),
                ballTransport.loadShooter(), ballTransport.moveBothMotorsToLaser(), new WaitCommand(0.5));
    }

    private CommandBase shootLow() {
        return sequence("Shoot Low", shooter.setTargetAndStartShooter(HubSpeed.LOW_HIGH_HOOD),
                shooter.waitUntilSpeedUp(),
                ballTransport.loadShooter(), ballTransport.moveBothMotorsToLaser());
    }

    private CommandBase shootOneBallAuto() {
        return shootHigh().withName("Shoot One Ball Auto");
    }

    private CommandBase shootTwoBallsAuto() {
        return sequence("Shoot Balls", shootHigh(), shootHigh(), stopShooter());
    }

    private CommandBase intakeOneBallAuto(double deploymentDelay) {
        return sequence("Intake One Ball", new WaitCommand(deploymentDelay),
                parallel("Intake and Tansport", intake.extend(SpinDirection.COUNTERCLOCKWISE),
                        race("Infinite Intake Stopper", ballTransport.moveLowerToColor(), new WaitCommand(1.5))),
                intake.retract(),
                race("Infinite Intake Stopper", ballTransport.moveLowerToColor(), new WaitCommand(.25)));
    }

    private CommandBase stopShooter() {
        return sequence("Stop Shooter", new WaitCommand(1), shooter.stop());
    }

    // Starting Against the right side of the hub, Shoot One Ball, Pickup 2 balls,
    // shoot them

    public CommandBase twoLeftAuto() {
        return sequence("Two Ball Left Auto", drive.resetAuto(AutoPaths.twoBallLeftOne),
                parallel("Intake and Drive", drive.auto(AutoPaths.twoBallLeftOne, 0.02), intakeOneBallAuto(1)),
                drive.auto(AutoPaths.twoBallLeftTwo,
                        0.23),
                shootTwoBallsAuto(), stopShooter());
    }

    public CommandBase weekTwoAutoHigh() {
        return sequence("Week Two Auto High",
                shootHigh(),

                parallel("Stop and drive",
                        sequence("Stop shooter", new WaitCommand(2), shooter.stop()),
                        drive.driveDistance(2.8, 0, 0.5)),
                parallel("Reset Arms", leftClimber.resetArms(), rightClimber.resetArms()));
    }

    public CommandBase delayedAuto() {
        return sequence("Delayed Auto",
                new WaitCommand(2),
                shootHigh(),

                parallel("Stop and drive",
                        sequence("Stop shooter", new WaitCommand(2), shooter.stop()),
                        drive.driveDistance(2.8, 0, 0.5)),
                parallel("Reset Arms", leftClimber.resetArms(), rightClimber.resetArms()));
    }

    public CommandBase weekTwoAutoLow() {
        return sequence("Week Two Auto Low",
                shootLow(),
                parallel("Stop and drive",
                        sequence("Stop shooter", new WaitCommand(2), shooter.stop()),
                        drive.driveDistance(2.8, 0, 0.5)),
                parallel("Reset Arms", leftClimber.resetArms(), rightClimber.resetArms()));
    }

    public CommandBase onlyShoot() {
        return sequence("Only Shoot", shootHigh());
    }
}
