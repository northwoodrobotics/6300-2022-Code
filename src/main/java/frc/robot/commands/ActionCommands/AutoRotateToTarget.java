package frc.robot.commands.ActionCommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.BlindLightCommands.LimelightSwitchLEDMode;
import frc.robot.commands.BlindLightCommands.LimelightWaitForTarget;
import frc.robot.commands.DriveCommands.AutonTurnToTarget;
import frc.robot.commands.DriveCommands.DriveAutoRotate;
import frc.robot.subsystems.Vision;
import frc.swervelib.SwerveSubsystem;
import frc.swervelib.SwerveSubsystem;
import frc.robot.Constants;

public class AutoRotateToTarget extends SequentialCommandGroup {

    public AutoRotateToTarget(SwerveSubsystem subsystem){
        addCommands(
            new LimelightSwitchLEDMode(Vision.LEDMode.LED_ON),
            new ParallelCommandGroup(
                new AutonTurnToTarget(subsystem),
                new LimelightWaitForTarget()
            )
        );

    }


    
}
