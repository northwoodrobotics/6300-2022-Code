package frc.robot.commands.AutoRoutines;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.PathHolder;
import frc.robot.commands.ActionCommands.DriveAndIntake;
import frc.robot.commands.DriveCommands.AutoDrive;
import frc.robot.commands.SimCommands.HoodUp;
import frc.robot.commands.SubsystemCommands.PurgeFeeder;
import frc.robot.commands.SubsystemCommands.ShooterCommand;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.Vision;
import frc.swervelib.SwerveSubsystem;

public class Friendly3Ball extends SequentialCommandGroup{

    public Friendly3Ball(SwerveSubsystem subsystem, ShooterSubsystem shooter, Vision blindlight, FeederSubsystem feeder, IntakeSubsystem intake){
        addCommands(
            new DriveAndIntake(PathHolder.Friendly3Ball, subsystem, intake, feeder),
            new HoodUp(shooter, 37).alongWith(new ShooterCommand(shooter, blindlight).alongWith(new WaitCommand(1.5), new PurgeFeeder(feeder, 1)))
            ,
            new DriveAndIntake(PathHolder.Friendly3BallPart2, subsystem, intake, feeder), 
            new AutoDrive(subsystem, PathHolder.Friendly3BallPart3),
            new HoodUp(shooter,37).alongWith(new ShooterCommand(shooter, blindlight).alongWith(new WaitCommand(1.5), new PurgeFeeder(feeder, 1)))
        );
    }
}
