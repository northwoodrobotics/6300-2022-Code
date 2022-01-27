package frc.robot.commands.AutoRoutines;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DriveCommands.*;
import frc.robot.PathHolder;
import frc.robot.RobotContainer;

public class JustSquare extends SequentialCommandGroup{
    
    public JustSquare(){
        addCommands(
            new AutoDrive(RobotContainer.m_swerveSubsystem, PathHolder.Square)
        );
    }
    
}
