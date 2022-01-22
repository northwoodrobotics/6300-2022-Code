package frc.robot.commands.AutoRoutines;



import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.AutoDrive;
import frc.robot.PathHolder;
public class RealSquare extends SequentialCommandGroup{


    public RealSquare(){
        addCommands(
            new AutoDrive(RobotContainer.m_swerveSubsystem, PathHolder.RealSquare)
        );

    }
    
}
