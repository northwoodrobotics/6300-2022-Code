package frc.robot.commands.SimCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;

public class HoodUp extends CommandBase{
    private ShooterSubsystem shooter; 
    private double newAngle;
    public HoodUp(ShooterSubsystem subsystem, double HoodAngle){
        shooter = subsystem;
        newAngle = HoodAngle;
    }
    @Override
    public void initialize() {
      //  newAngle = shooter.getHoodAngle()+1; 
    }
    @Override
    public void execute() {
        //subsystem.percentoutput(1);   
        //subsystem.RunShooter(Constants.ShooterConstants.ShooterVelocityTable.lookup(Blindight.getRobotToTargetDistance()));
        //subsystem.RunShooter(m_speed);
        //subsystem.MoveHood((Constants.ShooterConstants.HoodPositionTable.lookup(Math.round(Blindight.getAvgDistance() *10/10))));
        shooter.setHoodTargetAngle(newAngle);     
    }
    @Override
    public void end(boolean interrupted){
        

    }
    public boolean isFinsihed(){
        return shooter.isHoodAtTargetAngle();
    }
    

    
     
}
