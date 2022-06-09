package frc.robot.commands.SubsystemCommands.FeederCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
public class PurgeFeeder extends CommandBase{
    private final FeederSubsystem m_feeder;


    public PurgeFeeder(FeederSubsystem feeder){
        this.m_feeder = feeder;
      
    }
    
    @Override 
    public void initialize(){

        //m_intake.setIntakeExtension(true);

    }
    @Override
    public void execute(){
        //Timer.delay(1);
        m_feeder.SetFeed();
       

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
