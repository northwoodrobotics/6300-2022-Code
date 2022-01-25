package frc.robot.commands.AutoRoutines;


import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DriveCommands.*;
import frc.robot.PathHolder;
import frc.robot.RobotContainer;

public class DriveAndTurn extends SequentialCommandGroup{
    public DriveAndTurn(){
        addCommands(
            new AutoDrive(RobotContainer.m_swerveSubsystem, PathHolder.DriveAndTurn)
        );
    }
    
}
