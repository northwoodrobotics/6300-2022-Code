package frc.robot.commands.AutoRoutines;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.PathHolder;
import frc.robot.commands.DriveCommands.AutoDrive;
import frc.robot.commands.SimCommands.HoodUp;
import frc.robot.commands.SubsystemCommands.AutoShoot;
import frc.robot.commands.SubsystemCommands.PurgeFeeder;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.Vision;
import frc.swervelib.SwerveSubsystem;

public class OneBallRed extends SequentialCommandGroup{
    public OneBallRed(SwerveSubsystem subsystem, ShooterSubsystem shooter, Vision blindlight, FeederSubsystem feeder, IntakeSubsystem intake){
        addCommands(
            new AutoDrive(subsystem, PathHolder.OneBallRed), 
            new AutoShoot(shooter, blindlight).alongWith(new HoodUp(shooter,37), new PurgeFeeder(feeder, 0.5))
        );
    }
}
