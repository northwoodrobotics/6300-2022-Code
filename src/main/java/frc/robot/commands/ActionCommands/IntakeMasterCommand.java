package frc.robot.commands.ActionCommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.SubsystemCommands.IntakeCommand;
import frc.robot.commands.SubsystemCommands.RunFeeder;
import frc.robot.commands.SubsystemCommands.WaitForBall;
import frc.robot.commands.SubsystemCommands.WaitForSecondStage;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeMasterCommand extends ParallelCommandGroup{
    public IntakeMasterCommand(FeederSubsystem feeder, IntakeSubsystem intake){


        addCommands( new IntakeCommand(intake,0.7 ),
        new SortBall(feeder)
        //new SortBall(feeder)
           //new SortBall(feeder)
           
            
            
            
           
        );
}


    
}
