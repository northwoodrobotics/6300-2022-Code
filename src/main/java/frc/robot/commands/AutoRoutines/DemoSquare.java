package frc.robot.commands.AutoRoutines;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.AutoDrive;
import frc.robot.PathHolder;

public class DemoSquare extends SequentialCommandGroup{
    public DemoSquare(){
        addCommands(
            new AutoDrive(RobotContainer.m_swerveSubsystem, PathHolder.DemoSquare)
        );
    }
    
    
}
