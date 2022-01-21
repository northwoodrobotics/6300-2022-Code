package frc.robot.commands;


import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.swervelib.SwerveSubsystem;


public class AutoDrive extends SequentialCommandGroup{
    
    public AutoDrive(SwerveSubsystem subsystem, PathPlannerTrajectory trajectory){
        addCommands(new InstantCommand(() -> subsystem.dt.setKnownPose(trajectory.getInitialPose())),
        subsystem.dt.createCommandForTrajectory(trajectory, subsystem)
        );
            
    
        
    }
}
