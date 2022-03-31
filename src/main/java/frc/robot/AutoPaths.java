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
    public static final PathPlannerTrajectory twoBallRightOne = PathPlanner.loadPath("TwoBallRightOne", AUTO_SPEED,
            AUTO_ACCEL);
    public static final PathPlannerTrajectory twoBallRightTwo = PathPlanner.loadPath("TwoBallRightTwo", AUTO_SPEED,
            AUTO_ACCEL);
    public static final PathPlannerTrajectory twoBallLeftOne = PathPlanner.loadPath("TwoBallLeftOne", AUTO_SPEED,
            AUTO_ACCEL);
    public static final PathPlannerTrajectory twoBallLeftTwo = PathPlanner.loadPath("TwoBallLeftTwo", AUTO_SPEED,
            AUTO_ACCEL);
    public static final PathPlannerTrajectory oneBallLeftOne = PathPlanner.loadPath("OneBallLeftOne", AUTO_SPEED,
            AUTO_ACCEL);
}