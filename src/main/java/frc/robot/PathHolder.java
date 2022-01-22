package frc.robot;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

public class PathHolder {

    public static PathPlannerTrajectory FiveBall = PathPlanner.loadPath("Demo 5 Ball", 8, 5);
    public static PathPlannerTrajectory DemoSquare = PathPlanner.loadPath("DemoSquare", 8, 5);
    public static PathPlannerTrajectory RealSquare = PathPlanner.loadPath("RealSquare", 4, 3);
    
    
}
