package frc.robot.commands.AutoRoutines;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.PathHolder;
import frc.robot.commands.ActionCommands.DriveAndIntake;
import frc.robot.commands.BlindLightCommands.LimelightSwitchLEDMode;
import frc.robot.commands.SimCommands.HoodUp;
import frc.robot.commands.SubsystemCommands.PurgeFeeder;
import frc.robot.commands.SubsystemCommands.AutoCommands.AutoShoot;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.Vision;
import frc.swervelib.SwerveSubsystem;

public class WackyTwoBall extends SequentialCommandGroup{
   public  WackyTwoBall(SwerveSubsystem subsystem, ShooterSubsystem shooter, Vision blindlight, FeederSubsystem feeder, IntakeSubsystem intake){
        addCommands(
            
            new InstantCommand(() -> subsystem.dt.setKnownState(PathHolder.WackyTwoBall.getInitialState())),
            new LimelightSwitchLEDMode(Vision.LEDMode.LED_ON),
            new WaitCommand(3),
            new DriveAndIntake(PathHolder.WackyTwoBall, subsystem, intake, feeder),
            new ParallelDeadlineGroup(new AutoShoot(shooter, blindlight, 21.5, -5500), new SequentialCommandGroup(new WaitCommand(.8), new PurgeFeeder(feeder, 1)))
            //new AutoShoot(shooter, blindlight).alongWith(new HoodUp(shooter, 37), new PurgeFeeder(feeder, .75))
            
        );
   }
    
}
