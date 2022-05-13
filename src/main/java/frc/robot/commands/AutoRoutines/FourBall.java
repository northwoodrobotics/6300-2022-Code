package frc.robot.commands.AutoRoutines;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.PathHolder;
import frc.robot.commands.ActionCommands.DriveAndIntake;
import frc.robot.commands.ActionCommands.IndexToReady;
import frc.robot.commands.BlindLightCommands.LimelightSwitchLEDMode;
import frc.robot.commands.DriveCommands.AutoDrive;
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
    // actually a 3 ball auto. 

    public FourBall(SwerveSubsystem subsystem, ShooterSubsystem shooter, Vision blindlight, FeederSubsystem feeder, IntakeSubsystem intake){
        addCommands(
            new InstantCommand(() -> subsystem.dt.setKnownState(PathHolder.FourBall1.getInitialState())),
            new LimelightSwitchLEDMode(Vision.LEDMode.LED_ON),
            new DriveAndIntake(PathHolder.FourBall1, subsystem, intake, feeder), 
            new ParallelDeadlineGroup(new AutoShoot(shooter, blindlight, 21.5, -5500), new SequentialCommandGroup(new WaitCommand(.8), new PurgeFeeder(feeder, 1))),
            
            new DriveAndIntake(PathHolder.FourBall2, subsystem, intake, feeder),//.alongWith(new IndexToReady(feeder)),
            new ParallelCommandGroup(new AutoShoot(shooter, blindlight, 26.0,-7000), new SequentialCommandGroup(new WaitCommand(.8), new PurgeFeeder(feeder, 1))),
            new AutoDrive(subsystem, PathHolder.FourBall3)
            //new HoodUp(shooter,37).alongWith(new ShooterCommand(shooter, blindlight).alongWith(new WaitCommand(1.5), new PurgeFeeder(feeder, 1)))
        );
    }
}
