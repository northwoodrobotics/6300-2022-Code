package frc.robot.commands.SubsystemCommands;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeOutCommand extends CommandBase{
    private IntakeSubsystem m_intake; 
    public IntakeOutCommand(IntakeSubsystem intake){
        m_intake = intake;
    }
    public void initialize(){
        m_intake.setIntakeExtension(Value.kReverse);
        //m_intake.IntakeOut();
    }
    public void execute(){
        m_intake.setMotorOutput(0.45);
    }

    @Override 
    public void end(boolean interrupted){
        //m_intake.IntakeIn();
        m_intake.setIntakeExtension(Value.kForward);
        m_intake.setMotorOutput(0);
        //m_feeder.runFeeder(0.0);
        //m_intake.setMotorOutput(0.0);
        
    }
    
    
}
