package frc.robot.commands.SimCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DriveCommands.*;
import frc.swervelib.SwerveSubsystem;
import frc.robot.PathHolder;
import frc.robot.RobotContainer;
public class SimAuton extends SequentialCommandGroup{
    
    private final SwerveSubsystem swerveSubsystem;
    
    public SimAuton(SwerveSubsystem m_subsystem){
        this.swerveSubsystem = m_subsystem;
        addCommands(
            new AutoDrive(swerveSubsystem, PathHolder.simTrajectory)
        );
    }
}
