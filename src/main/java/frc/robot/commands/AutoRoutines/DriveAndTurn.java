package frc.robot.commands.AutoRoutines;


import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ActionCommands.AutoRotateToTarget;
import frc.robot.commands.DriveCommands.*;
import frc.swervelib.SwerveSubsystem;
import frc.robot.PathHolder;
import frc.robot.RobotContainer;

public class DriveAndTurn extends SequentialCommandGroup{
    
    
    public DriveAndTurn(SwerveSubsystem m_subsystem){
        
        addCommands(
            new InstantCommand(()-> m_subsystem.dt.setKnownPose(PathHolder.DriveAndTurn.getInitialPose())),
            m_subsystem.dt.createCommandForTrajectory(PathHolder.DriveAndTurn, m_subsystem),
           new  AutoDrive(m_subsystem, PathHolder.DriveAndTurn2)

            

            
        );
    }
    
}
