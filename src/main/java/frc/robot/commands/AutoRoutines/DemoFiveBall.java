package frc.robot.commands.AutoRoutines;


import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AutoDrive;
import frc.robot.PathHolder;
import frc.robot.RobotContainer;

public class DemoFiveBall extends SequentialCommandGroup{
    public DemoFiveBall(){
        addCommands(
            new AutoDrive(RobotContainer.m_swerveSubsystem, PathHolder.FiveBall)
        );
    }
    
}
