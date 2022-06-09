package frc.robot.commands.AutoRoutines;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.PathHolder;
import frc.robot.commands.ActionCommands.DriveAndIntake;
import frc.robot.commands.BlindLightCommands.LimelightSwitchLEDMode;
import frc.robot.commands.DriveCommands.AutoDrive;
//import frc.robot.commands.DriveCommands.AutonTurnToTarget;

import frc.robot.commands.SubsystemCommands.AutoCommands.AutoShoot;
import frc.robot.commands.SubsystemCommands.FeederCommands.PurgeFeeder;
import frc.robot.commands.SubsystemCommands.IntakeCommands.IntakeDeploy;
import frc.robot.commands.SubsystemCommands.ShooterCommands.ShooterCommand;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.Vision;
import frc.swervelib.SwerveSubsystem;

public class TwoBall extends SequentialCommandGroup{
    public TwoBall(SwerveSubsystem subsystem, ShooterSubsystem shooter, Vision blindlight, FeederSubsystem feeder, IntakeSubsystem intake){
        addCommands(    
            new InstantCommand(() -> subsystem.dt.setKnownState(PathHolder.TwoBall.getInitialState())),
            new LimelightSwitchLEDMode(Vision.LEDMode.LED_ON),
            new AutoDrive(subsystem, PathHolder.TwoBall), 
            new DriveAndIntake(PathHolder.TwoBall2, subsystem, intake, feeder),
            new ParallelCommandGroup(new AutoShoot(shooter, blindlight), new SequentialCommandGroup(new WaitCommand(1.5), new PurgeFeeder(feeder)))
           
                      
        );
    }
     
}
