package frc.robot.commands.AutoRoutines;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DriveCommands.*;
import frc.swervelib.SwerveSubsystem;
import frc.robot.PathHolder;
import frc.robot.RobotContainer;

public class JustSquare extends SequentialCommandGroup{
    private final SwerveSubsystem swerveSubsystem;
    
    public JustSquare(SwerveSubsystem m_subsystem){
        this.swerveSubsystem = m_subsystem;
        addCommands(
            new AutoDrive(swerveSubsystem, PathHolder.Square)
        );
    }
    
}
