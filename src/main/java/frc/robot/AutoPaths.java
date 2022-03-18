package frc.robot;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

public final class AutoPaths {
    public static final double AUTO_SPEED = 1;
    public static final double AUTO_ACCEL = .5;

    // Naming Scheme: Number of balls, Side of field, step number
    public static final PathPlannerTrajectory threeBallRightOne = PathPlanner.loadPath("ThreeBallAutoRightOne",
            AUTO_SPEED,
            AUTO_ACCEL);
    public static final PathPlannerTrajectory threeBallRightTwo = PathPlanner.loadPath("ThreeBallAutoRightTwo",
            AUTO_SPEED,
            AUTO_ACCEL);
    public static final PathPlannerTrajectory threeBallRightThree = PathPlanner.loadPath("ThreeBallAutoRightThree",
            AUTO_SPEED,
            AUTO_ACCEL);
    public static final PathPlannerTrajectory twoBallRightOne = PathPlanner.loadPath("TwoBallAutoRightOne", AUTO_SPEED,
            AUTO_ACCEL);
    public static final PathPlannerTrajectory twoBallRightTwo = PathPlanner.loadPath("TwoBallAutoRightTwo", AUTO_SPEED,
            AUTO_ACCEL);
    public static final PathPlannerTrajectory twoBallLeftOne = PathPlanner.loadPath("TwoBallAutoLeftOne", AUTO_SPEED,
            AUTO_ACCEL);
    public static final PathPlannerTrajectory twoBallLeftTwo = PathPlanner.loadPath("TwoBallAutoLeftTwo", AUTO_SPEED,
            AUTO_ACCEL);
    public static final PathPlannerTrajectory twoMeterX = PathPlanner.loadPath("TwoMeterX", AUTO_SPEED,
            AUTO_ACCEL);
    public static final PathPlannerTrajectory twoMeterY = PathPlanner.loadPath("TwoMeterY", AUTO_SPEED,
            AUTO_ACCEL);

}