package frc.robot.commands.SubsystemCommands.FeederCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.FeederSubsystem;

public class HoldAll extends CommandBase{
    private final FeederSubsystem m_feeder;

    public HoldAll(FeederSubsystem feeder){
        this.m_feeder = feeder;
      
    }
    @Override
    public void initialize() {
        
    }
    @Override
    public void execute() {
        m_feeder.SetAll();
    }
    @Override
    public void end(boolean interrupted) {
       m_feeder.SetIdle();
    }
    @Override
    public boolean isFinished() {
      return false;
    }



}
