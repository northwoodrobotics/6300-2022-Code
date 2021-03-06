package frc.robot.commands.AutoRoutines;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.PathHolder;
import frc.robot.commands.ActionCommands.DriveAndIntake;
import frc.robot.commands.SimCommands.HoodUp;
import frc.robot.commands.SubsystemCommands.PurgeFeeder;
import frc.robot.commands.SubsystemCommands.ShooterCommand;
import frc.robot.commands.SubsystemCommands.AutoCommands.AutoShoot;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.Vision;
import frc.swervelib.SwerveSubsystem;

public class FourBall extends SequentialCommandGroup{

    public FourBall(SwerveSubsystem subsystem, ShooterSubsystem shooter, Vision blindlight, FeederSubsystem feeder, IntakeSubsystem intake){
        addCommands(
            new DriveAndIntake(PathHolder.FourBall1, subsystem, intake, feeder),
            new AutoShoot(shooter, blindlight).alongWith(new HoodUp(shooter,27.5), new PurgeFeeder(feeder, 0.5))
            ,
            new DriveAndIntake(PathHolder.FourBall2, subsystem, intake, feeder),
            new AutoShoot(shooter, blindlight).alongWith(new HoodUp(shooter,27.5), new PurgeFeeder(feeder, 0.5))
            //new HoodUp(shooter,37).alongWith(new ShooterCommand(shooter, blindlight).alongWith(new WaitCommand(1.5), new PurgeFeeder(feeder, 1)))
        );
    }
}
