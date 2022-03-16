package frc.robot;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

public class PathHolder {

    public static PathPlannerTrajectory TwoBallRed = PathPlanner.loadPath("TwoBallRed", 1, 0.5);
    public static PathPlannerTrajectory TwoBallRed2 = PathPlanner.loadPath("TwoBallRed2", 1, 0.5);
    public static PathPlannerTrajectory TwoBall = PathPlanner.loadPath("TwoBall", 1, 0.5);
    public static PathPlannerTrajectory TwoBall2 = PathPlanner.loadPath("TwoBallPart2", 1, 0.5);
    public static PathPlannerTrajectory FourBall1 = PathPlanner.loadPath("FourBallPart1", 1, 0.5);
    public static PathPlannerTrajectory FourBall2 = PathPlanner.loadPath("FourBallPart2", 1, 0.5);
    public static PathPlannerTrajectory OneBallBlue = PathPlanner.loadPath("BlueHappy1Ball", 1, 0.5);
    public static PathPlannerTrajectory OneBallRed = PathPlanner.loadPath("RedHappy1Ball", 1, 0.5);
}
