package frc.robot.commands.ActionCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.SubsystemCommands.RunFeeder;
import frc.robot.commands.SubsystemCommands.WaitForBall;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class SortBall extends SequentialCommandGroup{
    public SortBall(FeederSubsystem feeder){
        addCommands(
            new WaitForBall(feeder), new RunFeeder(feeder), new WaitForBall(feeder),new RunFeeder(feeder)
        );
        
    }
    
}
