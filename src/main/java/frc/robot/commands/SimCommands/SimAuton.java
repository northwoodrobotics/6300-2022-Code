package frc.robot.commands.SimCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DriveCommands.*;
import frc.robot.PathHolder;
import frc.robot.RobotContainer;
public class SimAuton extends SequentialCommandGroup{
    
    public SimAuton(){
        addCommands(
            new AutoDrive(RobotContainer.m_swerveSubsystem, PathHolder.simTrajectory)
        );
    }
    
}
