package frc.robot.commands.AutoRoutines;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.PathHolder;
import frc.robot.commands.ActionCommands.DriveAndIntake;
import frc.robot.commands.DriveCommands.AutonTurnToTarget;
import frc.robot.commands.SubsystemCommands.PurgeFeeder;
import frc.robot.commands.SubsystemCommands.ShooterCommand;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.Vision;
import frc.swervelib.SwerveSubsystem;

public class TwoBall extends SequentialCommandGroup{
    public TwoBall(SwerveSubsystem subsystem, ShooterSubsystem shooter, Vision blindlight, FeederSubsystem feeder, IntakeSubsystem intake){
        addCommands(
            new DriveAndIntake(PathHolder.TwoBall, subsystem, intake, feeder), 
            new AutonTurnToTarget(subsystem).alongWith(new ShooterCommand(shooter, blindlight)).alongWith(
                new WaitCommand(0.25).andThen(new PurgeFeeder(feeder, 0.5))
            )
        );
    }
    
}
