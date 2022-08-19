package frc.robot.commands.SubsystemCommands.FeederCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.FeederSubsystem;

public class EmptyFeeder extends CommandBase{
    private final FeederSubsystem m_feeder;


    public EmptyFeeder(FeederSubsystem feeder){
        this.m_feeder = feeder;
      
    }
    
    @Override 
    public void initialize(){

        //m_intake.setIntakeExtension(true);

    }
    @Override
    public void execute(){
        //Timer.delay(1);
        m_feeder.SetEmpty();
       

    }
    @Override 
    public void end(boolean interrupted){
        m_feeder.SetIdle();
        //m_intake.setMotorOutput(0.0);
        
    
    }
    @Override 
    public boolean isFinished(){
        return (!m_feeder.BallAtEjector()&& !m_feeder.Stage2Loaded());
    }
}
