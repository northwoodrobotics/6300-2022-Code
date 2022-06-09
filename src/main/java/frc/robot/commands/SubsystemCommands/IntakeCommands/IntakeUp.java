package frc.robot.commands.SubsystemCommands.IntakeCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.IntakeSubsystem;


public class IntakeUp extends CommandBase{
    private final IntakeSubsystem m_intake;
    
    private final double IntakeSpeed;

    
    public IntakeUp(IntakeSubsystem intake, double intakeSpeed){
            this.IntakeSpeed = intakeSpeed;
           
            this.m_intake = intake;

            addRequirements(intake);



    }
    
    @Override 
    public void initialize(){
        m_intake.IntakeUp();
        

    }
    
    @Override
    public void execute(){
        /*if(m_feeder.Stage2Loaded()){
            m_intake.setMotorOutput(-IntakeSpeed);
            m_feeder.runFeeder(0);
        }
        else if(!m_feeder.Stage2Loaded()){
            m_intake.setMotorOutput(IntakeSpeed);
            //m_intake.setIntakeExtension(Value.kReverse);
        }
        if(m_feeder.shouldAdvance()){
            feedTimer.start();
        
            //Timer.delay(0.3);
            //new WaitCommand(0.5);
            m_feeder.runFeeder(.6);
            m_intake.setMotorOutput(0.3);
            //new WaitCommand(0.5);
     
        }*/
        //else m_feeder.runFeeder(0);


            
       

        //m_intake.setIntakeExtension(Value.kForward);
       

    }
    @Override 
    public void end(boolean interrupted){
        m_intake.IntakeDribble();
        
    }


    
    
}
