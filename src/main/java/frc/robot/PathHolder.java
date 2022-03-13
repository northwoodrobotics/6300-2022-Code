package frc.robot;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

public class PathHolder {

    public static PathPlannerTrajectory TwoBall = PathPlanner.loadPath("TwoBall", 1, 0.5);
    public static PathPlannerTrajectory TwoBall2 = PathPlanner.loadPath("TwoBallPart2", 1, 0.5);
}
