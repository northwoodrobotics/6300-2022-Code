package frc.robot.commands.AutoRoutines;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.PathHolder;
import frc.robot.commands.DriveCommands.AutoDrive;
import frc.robot.commands.SimCommands.HoodUp;
import frc.robot.commands.SubsystemCommands.PurgeFeeder;
import frc.robot.commands.SubsystemCommands.AutoCommands.AutoShoot;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.Vision;
import frc.swervelib.SwerveSubsystem;

public class OneBall extends SequentialCommandGroup{
    public OneBall(SwerveSubsystem subsystem, ShooterSubsystem shooter, Vision blindlight, FeederSubsystem feeder, IntakeSubsystem intake){
        addCommands(
            new AutoDrive(subsystem, PathHolder.OneBallBlue), 
            new AutoShoot(shooter, blindlight).alongWith(new HoodUp(shooter,23.5), new SequentialCommandGroup( new WaitCommand(1.5), new PurgeFeeder(feeder, 1)))
        );
    }
}
