package frc.robot.commands.AutoRoutines;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.PathHolder;
import frc.robot.commands.ActionCommands.DriveAndIntake;
//import frc.robot.commands.ActionCommands.IndexToReady;
import frc.robot.commands.BlindLightCommands.LimelightSwitchLEDMode;
import frc.robot.commands.DriveCommands.AutoDrive;
import frc.robot.commands.SubsystemCommands.AutoCommands.AutoShoot;
import frc.robot.commands.SubsystemCommands.AutoCommands.CheckVelocity;
import frc.robot.commands.SubsystemCommands.FeederCommands.PurgeFeeder;
//import frc.robot.commands.SubsystemCommands.ShooterCommands.ShooterCommand;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.Vision;
import frc.swervelib.SwerveSubsystem;

public class FourBall extends SequentialCommandGroup{
 

    public FourBall(SwerveSubsystem subsystem, ShooterSubsystem shooter, Vision blindlight, FeederSubsystem feeder, IntakeSubsystem intake){
        addCommands(
            new InstantCommand(() -> subsystem.dt.setKnownState(PathHolder.FourBall1.getInitialState())),
            new LimelightSwitchLEDMode(Vision.LEDMode.LED_ON),
            new DriveAndIntake(PathHolder.FourBall1, subsystem, intake, feeder), 
            new ParallelDeadlineGroup(new SequentialCommandGroup(new CheckVelocity(), new PurgeFeeder(feeder))),
            
            new DriveAndIntake(PathHolder.FourBall2, subsystem, intake, feeder),
            new WaitCommand(0.9),//.alongWith(new IndexToReady(feeder)),
            new AutoDrive(subsystem, PathHolder.FourBall3),
            new ParallelCommandGroup( new SequentialCommandGroup(new CheckVelocity(), new PurgeFeeder(feeder)))
           
        );
    }
}
