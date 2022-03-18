package frc.robot.commands.AutoRoutines;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.PathHolder;
import frc.robot.commands.ActionCommands.DriveAndIntake;
import frc.robot.commands.DriveCommands.AutoDrive;
import frc.robot.commands.DriveCommands.AutonTurnToTarget;
import frc.robot.commands.SimCommands.HoodUp;
import frc.robot.commands.SubsystemCommands.IntakeCommand;
//import frc.robot.commands.SubsystemCommands.IntakeOutCommand;
import frc.robot.commands.SubsystemCommands.PurgeFeeder;
import frc.robot.commands.SubsystemCommands.ShooterCommand;
import frc.robot.commands.SubsystemCommands.AutoCommands.AutoShoot;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.Vision;
import frc.swervelib.SwerveSubsystem;

public class TwoBall extends SequentialCommandGroup{
    public TwoBall(SwerveSubsystem subsystem, ShooterSubsystem shooter, Vision blindlight, FeederSubsystem feeder, IntakeSubsystem intake){
        addCommands(
            new AutoDrive(subsystem, PathHolder.TwoBall), 
            //new ShooterCommand(shooter, blindlight).alongWith(new PurgeFeeder(feeder, 0.3)),
            new DriveAndIntake(PathHolder.TwoBall2, subsystem, intake, feeder),
            new AutoShoot(shooter, blindlight).alongWith(new HoodUp(shooter, 37)).alongWith( new WaitCommand(2.0), new PurgeFeeder(feeder, 0.5))
            
            //new AutoDrive(subsystem, PathHolder.TwoBall2).alongWith(new IntakeOutCommand(intake), new PurgeFeeder(feeder, 0.35), new ShooterCommand(shooter, blindlight) )

           // new AutoDrive(subsystem, PathHolder.TwoBall2).alongWith(new IntakeOutCommand(intake)), 
           // new ShooterCommand(shooter, blindlight).alongWith(new PurgeFeeder(feeder, 0.6))
            
            
            /*new AutonTurnToTarget(subsystem).alongWith(new ShooterCommand(shooter, blindlight)).alongWith(
                new WaitCommand(0.25).andThen(new PurgeFeeder(feeder, 0.5))*/
            
                      
        );
    }
     
}
