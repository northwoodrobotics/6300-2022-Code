package frc.robot;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

public class PathHolder {

    public static PathPlannerTrajectory DriveAndTurn = PathPlanner.loadPath("DriveAndTurn", 1, 0.5);
    public static PathPlannerTrajectory DemoSquare = PathPlanner.loadPath("DemoSquare", 4, 1);
    public static PathPlannerTrajectory RealSquare = PathPlanner.loadPath("RealSquare", 4, 2);
    public static PathPlannerTrajectory Square = PathPlanner.loadPath("Square", 2, 1);
    public static PathPlannerTrajectory simTrajectory = PathPlanner.loadPath("Test the PID LOOP", 2, 1);
    public static PathPlannerTrajectory DriveAndGoLeft = PathPlanner.loadPath("Training Path", 2,1 );
    public static PathPlannerTrajectory DriveAndTurn2 = PathPlanner.loadPath("nope", 1, 0.5);
    
}
