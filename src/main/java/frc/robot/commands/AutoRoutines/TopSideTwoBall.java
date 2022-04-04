package frc.robot.commands.AutoRoutines;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.PathHolder;
import frc.robot.commands.ActionCommands.DriveAndIntake;
import frc.robot.commands.SimCommands.HoodUp;
import frc.robot.commands.SubsystemCommands.PurgeFeeder;
import frc.robot.commands.SubsystemCommands.AutoCommands.AutoShoot;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.Vision;
import frc.swervelib.SwerveSubsystem;

public class TopSideTwoBall extends SequentialCommandGroup{
    public TopSideTwoBall(SwerveSubsystem subsystem, ShooterSubsystem shooter, Vision blindlight, FeederSubsystem feeder, IntakeSubsystem intake){
        addCommands(
            new InstantCommand(() -> subsystem.dt.setKnownState(PathHolder.FourBall1.getInitialState())),
            new DriveAndIntake(PathHolder.FourBall1, subsystem, intake, feeder), 
            new ParallelCommandGroup(new AutoShoot(shooter, blindlight, 21.5, -5500), new SequentialCommandGroup(new WaitCommand(0.65), new PurgeFeeder(feeder, 1)))
            //new ParallelCommandGroup(new AutoShoot(shooter, blindlight), new HoodUp(shooter, 23.5), new SequentialCommandGroup(new WaitCommand(1.5), new PurgeFeeder(feeder, 1)))

            //new InstantCommand(()-> subsystem.dt.plusNinetyGyroscope())




        );
    }   
}
