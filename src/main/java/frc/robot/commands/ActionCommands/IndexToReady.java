package frc.robot.commands.ActionCommands;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.SubsystemCommands.PurgeFeeder;
import frc.robot.commands.SubsystemCommands.WaitForSecondStage;
import frc.robot.commands.SubsystemCommands.AutoCommands.WaitFor2Balls;
import frc.robot.subsystems.FeederSubsystem;

public class IndexToReady extends SequentialCommandGroup{
    public IndexToReady(FeederSubsystem feeder){
        addCommands(
            new ParallelDeadlineGroup(new WaitForSecondStage(feeder), new PurgeFeeder(feeder, 1))
        );

    }
    
}
