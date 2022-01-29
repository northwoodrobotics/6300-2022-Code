package frc.robot.commands.AutoRoutines;



import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.DriveCommands.*;
import frc.swervelib.SwerveSubsystem;
import frc.robot.PathHolder;
public class RealSquare extends SequentialCommandGroup{


    private final SwerveSubsystem swerveSubsystem;
    
    public RealSquare(SwerveSubsystem m_subsystem){
        this.swerveSubsystem = m_subsystem;
        addCommands(
            new AutoDrive(swerveSubsystem, PathHolder.RealSquare)
        );
    }
}
