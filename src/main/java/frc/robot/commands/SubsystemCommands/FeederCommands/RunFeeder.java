package frc.robot.commands.SubsystemCommands.FeederCommands;


import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.FeederSubsystem;

public class RunFeeder extends CommandBase{
    // runs feeder for half a second 
    Timer timer;
    FeederSubsystem m_feeder; 
    public RunFeeder(FeederSubsystem feeder){
        m_feeder = feeder;
        //addRequirements(feeder);
        timer = new Timer(); 
    }
    @Override
    public void initialize() {
        m_feeder.SetFeederMode();
        
    }
    @Override
    public void execute() {
        m_feeder.SetIndex();
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
