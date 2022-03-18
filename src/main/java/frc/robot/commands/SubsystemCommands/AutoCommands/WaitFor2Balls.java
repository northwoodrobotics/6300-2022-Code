package frc.robot.commands.SubsystemCommands.AutoCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.IntakeSubsystem;
import kotlin.internal.OnlyInputTypes;
import frc.robot.subsystems.FeederSubsystem;
//import frc.ExternalLib.SpectrumLib.controllers.SpectrumXboxController;

public class WaitFor2Balls extends CommandBase{
    private final IntakeSubsystem m_intake;
    private final FeederSubsystem m_feeder;
    //private SpectrumXboxController controller;
    private final double IntakeSpeed;
    private Timer feedTimer; 
    //private double speed;
    
    public WaitFor2Balls(IntakeSubsystem intake,  FeederSubsystem feeder,double intakeSpeed){
            this.IntakeSpeed = intakeSpeed;
            //this.controller = controller;
            this.m_feeder = feeder;
            this.m_intake = intake;
            //this.speed = intakeSpeed;
            feedTimer = new Timer();
            
            addRequirements(intake);



    }

    @Override 
    public void initialize(){
        m_intake.setIntakeExtension(Value.kReverse);

    }
    @Override
    public void execute(){
        if(m_feeder.Stage2Loaded()){
            m_intake.setMotorOutput(-IntakeSpeed);
            m_feeder.runFeeder(0);
        }
        else if(!m_feeder.Stage2Loaded()){
            m_intake.setMotorOutput(IntakeSpeed);
            //m_intake.setIntakeExtension(Value.kReverse);
        }
        if(m_feeder.shouldAdvance()){
            Timer.delay(0.5);
            m_feeder.runFeeder(.7);
            m_intake.setMotorOutput(0.3);
            Timer.delay(0.4);
        }else m_feeder.runFeeder(0);


            
       

        //m_intake.setIntakeExtension(Value.kForward);
        m_intake.setMotorOutput(IntakeSpeed);
        

    }
    @Override 
    public void end(boolean interrupted){
        m_intake.setIntakeExtension(Value.kForward);
        //m_feeder.runFeeder(0.0);
        m_intake.setMotorOutput(0.0);
        
    }
    @Override 
    public boolean isFinished(){
        return m_feeder.Stage2Loaded();
    }

    
}



    
    

