package frc.robot;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

public class PathHolder {

    public static PathPlannerTrajectory DriveAndTurn = PathPlanner.loadPath("DriveAndTurn", 2, 1);
    public static PathPlannerTrajectory DemoSquare = PathPlanner.loadPath("DemoSquare", 8, 5);
    public static PathPlannerTrajectory RealSquare = PathPlanner.loadPath("RealSquare", 2, 1);
    
    
}
