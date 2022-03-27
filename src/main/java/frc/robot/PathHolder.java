package frc.robot;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.util.Units;

public class PathHolder {

    //public static PathPlannerTrajectory TwoBallRed = PathPlanner.loadPath("TwoBallRed", 1, 0.5);
    //public static PathPlannerTrajectory TwoBallRed2 = PathPlanner.loadPath("TwoBallRed2", 1, 0.5);
    public static PathPlannerTrajectory TwoBall = PathPlanner.loadPath("TwoBall", 1, 0.5);
    public static PathPlannerTrajectory TwoBall2 = PathPlanner.loadPath("TwoBallPart2", 1, 0.5);
    public static PathPlannerTrajectory TwoBallExtended = PathPlanner.loadPath("2BallExtended", 1, 0.5);
    public static PathPlannerTrajectory FourBall1 = PathPlanner.loadPath("FourBallPart1", 1, 0.5);
    public static PathPlannerTrajectory FourBall2 = PathPlanner.loadPath("FourBallPart2", 1, 0.5);
    public static PathPlannerTrajectory OneBallBlue = PathPlanner.loadPath("BlueHappy1Ball", Units.feetToMeters(1), Units.feetToMeters(1));
    public static PathPlannerTrajectory WackyTwoBall = PathPlanner.loadPath("Wacky2Ball", 1, 0.5);
    public static PathPlannerTrajectory Friendly3Ball = PathPlanner.loadPath("Friendly3BallPart1", 1, 0.5);
    public static PathPlannerTrajectory Friendly3BallPart2 = PathPlanner.loadPath("Friendly3BallPart2", 1, 0.5);
    public static PathPlannerTrajectory Friendly3BallPart3 = PathPlanner.loadPath("Friendly3BallPart3", 1, 0.5);
    //public static PathPlannerTrajectory OneBallRed = PathPlanner.loadPath("RedHappy1Ball", 1, 0.5);
}
