package frc.robot;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.util.Units;

public class PathHolder {

  public static PathPlannerTrajectory TwoBall = PathPlanner.loadPath("TwoBall", 1, 0.5);
    public static PathPlannerTrajectory TwoBall2 = PathPlanner.loadPath("TwoBallPart2", 1, 0.5);
    public static PathPlannerTrajectory TwoBallExtended = PathPlanner.loadPath("2BallExtended", 3, 1.5);
    public static PathPlannerTrajectory FourBall1 = PathPlanner.loadPath("FourBallPart1", 4, 2);
    public static PathPlannerTrajectory FourBall2 = PathPlanner.loadPath("FourBallPart2", 4, 2);
    public static PathPlannerTrajectory FourBall3 = PathPlanner.loadPath("FourBallPart3", 4, 3);
    public static PathPlannerTrajectory OneBallBlue = PathPlanner.loadPath("BlueHappy1Ball", Units.feetToMeters(1), Units.feetToMeters(1));
    public static PathPlannerTrajectory WackyTwoBall = PathPlanner.loadPath("Wacky2Ball", 2, 1);
    public static PathPlannerTrajectory Friendly3Ball = PathPlanner.loadPath("Friendly3BallPart1", 1, 0.5);
    public static PathPlannerTrajectory Friendly3BallPart2 = PathPlanner.loadPath("Friendly3BallPart2", 1, 0.5);
    public static PathPlannerTrajectory Friendly3BallPart3 = PathPlanner.loadPath("Friendly3BallPart3", 1, 0.5);
  
}
