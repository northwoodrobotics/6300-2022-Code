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
    public static PathPlannerTrajectory BallSteal = PathPlanner.loadPath("Le3Ball", 4, 0.5);
    public static PathPlannerTrajectory BallStealPart2 = PathPlanner.loadPath("Le3BallPart2", 4, 0.5);
    public static PathPlannerTrajectory BallStealPart3 = PathPlanner.loadPath("Le3BallPart3", 4, 0.5);
    public static PathPlannerTrajectory SixBallPart1 = PathPlanner.loadPath("Six+Half Part 1", 4, 0.5);
    public static PathPlannerTrajectory SixBallPart2 = PathPlanner.loadPath("Six+Half Part 2", 4, 0.5);
    

  
}
