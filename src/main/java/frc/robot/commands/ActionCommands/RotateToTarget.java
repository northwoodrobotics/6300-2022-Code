package frc.robot.commands.ActionCommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.BlindLightCommands.LimelightSwitchLEDMode;
import frc.robot.commands.BlindLightCommands.LimelightWaitForTarget;
import frc.robot.commands.DriveCommands.DriveAutoRotate;
import frc.robot.subsystems.Vision;
import frc.swervelib.SwerveSubsystem;
import frc.swervelib.SwerveSubsystem;
import frc.robot.Constants;
import java.util.function.DoubleSupplier;

public class RotateToTarget extends SequentialCommandGroup{

   public RotateToTarget(SwerveSubsystem subsystem, DoubleSupplier translationXSupplier,
    DoubleSupplier translationYSupplier,
    DoubleSupplier rotationSupplier){
       addCommands(
           new LimelightSwitchLEDMode(Vision.LEDMode.LED_ON),
           new ParallelCommandGroup(
            new LimelightWaitForTarget(),
            
                
               new DriveAutoRotate(subsystem, translationXSupplier, translationYSupplier)
           )
       );
   }


    
}
