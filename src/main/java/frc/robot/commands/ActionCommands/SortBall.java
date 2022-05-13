package frc.robot.commands.ActionCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.SubsystemCommands.RunFeeder;
import frc.robot.commands.SubsystemCommands.WaitForBall;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class SortBall extends SequentialCommandGroup{
    // this is a limitation of the physical 2022 robot, we cannot run a native state machine due to a delay requriement of the indexer. After 2 balls have been indexted, the driver must reset the command by actuating the intake. 
    public SortBall(FeederSubsystem feeder){
        addCommands(
            new WaitForBall(feeder), new RunFeeder(feeder), new WaitForBall(feeder),new RunFeeder(feeder)
        );
        
    }
    
}
