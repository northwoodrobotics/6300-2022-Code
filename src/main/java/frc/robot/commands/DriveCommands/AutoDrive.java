package frc.robot.commands.DriveCommands;


import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.swervelib.SwerveSubsystem;


public class AutoDrive extends SequentialCommandGroup{
    // feeds a trajectory into the pathfollowing. 
    
    public AutoDrive(SwerveSubsystem subsystem, PathPlannerTrajectory trajectory){
        addCommands(
        subsystem.dt.createCommandForTrajectory(trajectory, subsystem)
        );
            
    
        
    }
}
