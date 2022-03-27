package frc.robot.commands.ActionCommands;

import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DriveCommands.AutoDrive;
import frc.robot.commands.SubsystemCommands.IntakeCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.swervelib.SwerveSubsystem;

public class DriveAndIntake extends ParallelDeadlineGroup{
    public DriveAndIntake(PathPlannerTrajectory path, SwerveSubsystem swerve, IntakeSubsystem intake, FeederSubsystem feeder){
        super(
            new SequentialCommandGroup(
                new AutoDrive(swerve,path)
            ), new IntakeMasterCommand(feeder, intake)
        );

    }
    
}
