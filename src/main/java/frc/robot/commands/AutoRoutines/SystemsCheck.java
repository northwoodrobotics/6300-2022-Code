package frc.robot.commands.AutoRoutines;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.PathHolder;
import frc.robot.commands.ActionCommands.DriveAndIntake;
import frc.robot.commands.SimCommands.HoodUp;
import frc.robot.commands.SimCommands.TuneTables;
import frc.robot.commands.SubsystemCommands.AutoCommands.AutoShoot;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.Vision;
import frc.swervelib.SwerveSubsystem;

public class SystemsCheck extends SequentialCommandGroup{
   public  SystemsCheck(SwerveSubsystem subsystem, ShooterSubsystem shooter, Vision blindlight, FeederSubsystem feeder, IntakeSubsystem intake){
       addCommands(
           new ParallelCommandGroup(new DriveAndIntake(PathHolder.FourBall1, subsystem, intake, feeder), 
           new HoodUp(shooter, 40)), 
           new DriveAndIntake(PathHolder.FourBall2, subsystem, intake, feeder), 
           new HoodUp(shooter, 4).alongWith(new AutoShoot( shooter, blindlight))
           
       );

    }


}
