package frc.robot.commands.AutoRoutines;


import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ActionCommands.AutoRotateToTarget;
import frc.robot.commands.DriveCommands.*;
import frc.swervelib.SwerveSubsystem;
import frc.robot.PathHolder;
import frc.robot.RobotContainer;

public class DriveAndTurn extends SequentialCommandGroup{
    private final SwerveSubsystem swerveSubsystem;
    public DriveAndTurn(SwerveSubsystem m_subsystem){
        this.swerveSubsystem = m_subsystem;
        addCommands(
            new AutoDrive(swerveSubsystem, PathHolder.DriveAndTurn),
            new AutoRotateToTarget(swerveSubsystem)
        );
    }
    
}
