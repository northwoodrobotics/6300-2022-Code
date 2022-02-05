package frc.robot;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

public class PathHolder {

    public static PathPlannerTrajectory DriveAndTurn = PathPlanner.loadPath("DriveAndTurn", 4, 2);
    public static PathPlannerTrajectory DemoSquare = PathPlanner.loadPath("DemoSquare", 8, 5);
    public static PathPlannerTrajectory RealSquare = PathPlanner.loadPath("RealSquare", 4, 2);
    public static PathPlannerTrajectory Square = PathPlanner.loadPath("Square", 2, 1);
    public static PathPlannerTrajectory simTrajectory = PathPlanner.loadPath("Test the PID LOOP", 2, 1);
    public static PathPlannerTrajectory DriveAndGoLeft = PathPlanner.loadPath("Training Path", 2,1 );
    
}
