package frc.robot.commands.AutoRoutines;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.PathHolder;
import frc.robot.commands.ActionCommands.DriveAndIntake;
import frc.robot.commands.DriveCommands.AutoDrive;

import frc.robot.commands.SubsystemCommands.FeederCommands.PurgeFeeder;
import frc.robot.commands.SubsystemCommands.ShooterCommands.ShooterCommand;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.Vision;
import frc.swervelib.SwerveSubsystem;

public class Friendly3Ball extends SequentialCommandGroup{

    public Friendly3Ball(SwerveSubsystem subsystem, ShooterSubsystem shooter, Vision blindlight, FeederSubsystem feeder, IntakeSubsystem intake){
        addCommands(
            new DriveAndIntake(PathHolder.TwoBall, subsystem, intake, feeder),
            new PurgeFeeder(feeder)

         
        );
    }
}
