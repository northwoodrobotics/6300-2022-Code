package frc.robot.commands.SubsystemCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
public class PurgeFeeder extends CommandBase{
    private final FeederSubsystem m_feeder;
    private final double speed; 
    private Timer PurgeTimer;
    public PurgeFeeder(FeederSubsystem feeder, double FeederSpeed ){
        this.m_feeder = feeder;
        this.speed = FeederSpeed;
        PurgeTimer = new Timer();
        
    }
    
    @Override 
    public void initialize(){
        PurgeTimer.reset();
        PurgeTimer.start();
        //m_intake.setIntakeExtension(true);

    }
    @Override
    public void execute(){
        //Timer.delay(1);
        m_feeder.runFeeder(speed);
       

    }
    @Override 
    public void end(boolean interrupted){
        m_feeder.runFeeder(0.0);
        //m_intake.setMotorOutput(0.0);
        
    
    }
    @Override 
    public boolean isFinished(){
        return PurgeTimer.get() > 2;
    }

}
