package frc.robot.commands.SubsystemCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.FeederSubsystem;

public class WaitForBall extends CommandBase{
    // waits for the intake to recive a ball
    private FeederSubsystem m_feeder; 
    public WaitForBall(FeederSubsystem feeder){
        m_feeder = feeder;
        //addRequirements(feeder);
    }
    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
    }
    @Override
    public boolean isFinished() {
        return m_feeder.shouldAdvance();
    }
    
}
