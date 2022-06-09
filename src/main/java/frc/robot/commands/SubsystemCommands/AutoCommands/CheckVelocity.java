package frc.robot.commands.SubsystemCommands.AutoCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class CheckVelocity extends CommandBase{
    
    public CheckVelocity() {
        addRequirements(RobotContainer.shooter);
        
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
        return RobotContainer.shooter.isFlyWheelAtTargetVelocity();
    }
    
}
