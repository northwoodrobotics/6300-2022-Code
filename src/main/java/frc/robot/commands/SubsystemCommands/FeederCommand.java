package frc.robot.commands.SubsystemCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
public class FeederCommand extends CommandBase{
    private final FeederSubsystem m_feeder;
    private final double speed; 
    public FeederCommand(FeederSubsystem feeder, double FeederSpeed ){
        this.m_feeder = feeder;
        this.speed = FeederSpeed;
    }
    
    @Override 
    public void initialize(){
        //m_intake.setIntakeExtension(true);

    }
    @Override
    public void execute(){
        m_feeder.runFeeder(speed);
       

    }
    @Override 
    public void end(boolean interrupted){
        m_feeder.runFeeder(0.0);
        //m_intake.setMotorOutput(0.0);
        
    }

}
