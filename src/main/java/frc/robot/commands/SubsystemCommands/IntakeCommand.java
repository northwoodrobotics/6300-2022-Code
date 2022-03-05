package frc.robot.commands.SubsystemCommands;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.IntakeSubsystem;

import frc.robot.subsystems.FeederSubsystem;
//import frc.ExternalLib.SpectrumLib.controllers.SpectrumXboxController;

public class IntakeCommand extends CommandBase{
    private final IntakeSubsystem m_intake;
    //private final FeederSubsystem m_feeder;
    //private SpectrumXboxController controller;
    private final double IntakeSpeed;
    //private double speed;

    public IntakeCommand(IntakeSubsystem intake,  double intakeSpeed){
            this.IntakeSpeed = intakeSpeed;
            //this.controller = controller;
            //this.m_feeder = feeder;
            this.m_intake = intake;
            //this.speed = intakeSpeed;
            
            addRequirements(intake);



    }

    @Override 
    public void initialize(){
        //m_intake.setIntakeExtension(Value.kReverse);

    }
    @Override
    public void execute(){
        /*if(m_feeder.Stage2Loaded()){
            m_intake.setMotorOutput(0);
        }
        else if(!m_feeder.Stage2Loaded()){
            m_intake.setMotorOutput(IntakeSpeed);
            m_intake.setIntakeExtension(Value.kReverse);
        }
        if(m_feeder.shouldAdvance()){
            m_feeder.runFeeder(speed);
        }else{
            m_feeder.runFeeder(0);
        }*/
        //m_intake.setIntakeExtension(Value.kForward);
        m_intake.setMotorOutput(IntakeSpeed);
        

    }
    @Override 
    public void end(boolean interrupted){
        //m_intake.setIntakeExtension(Value.kForward);
        //m_feeder.runFeeder(0.0);
        m_intake.setMotorOutput(0.0);
        
    }

    
}



    
    

