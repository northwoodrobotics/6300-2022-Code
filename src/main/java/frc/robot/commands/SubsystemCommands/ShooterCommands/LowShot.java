package frc.robot.commands.SubsystemCommands.ShooterCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.Vision;

public class LowShot extends CommandBase{
    private final ShooterSubsystem subsystem; 
    private final Vision Blindight; 
    private double speed;
    private Timer timer;

public LowShot(ShooterSubsystem shooter, Vision blindlight){
    this.subsystem = shooter;
    //this.speed = speed;
    this.Blindight = blindlight;
  

    addRequirements(shooter);}
    @Override
public void initialize() {
    subsystem.setFlywheelCurrentLimitEnabled(false);

}
@Override
public void execute() {
    //subsystem.percentoutput(1);   
    
    //subsystem.RunShooter(Constants.ShooterConstants.ShooterVelocityTable.lookup(Blindight.getRobotToTargetDistance()));
    subsystem.RunShooter(-2500);
    subsystem.setHoodTargetAngle(35.5);
    //subsystem.MoveHood((Constants.ShooterConstants.HoodPositionTable.lookup(Math.round(Blindight.getAvgDistance() *10/10))));
    //subsystem.setHoodTargetAngle((Constants.ShooterConstants.HoodPositionTable.lookup(Math.round(Blindight.getAvgDistance() *10/10))));  
       
}

@Override
public void end(boolean interrupted) {
    subsystem.setFlywheelCurrentLimitEnabled(true);
    subsystem.stopFlywheel();
   
}

    
}
