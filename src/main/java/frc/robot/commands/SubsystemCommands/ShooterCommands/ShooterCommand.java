package frc.robot.commands.SubsystemCommands.ShooterCommands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ShooterConstants;
//import frc.ExternalLib.CitrusLib.CitrusConstants.ShooterConstants;
//import frc.ExternalLib.JackInTheBotLib.util.InterpolatingDouble;
//import frc.ExternalLib.JackInTheBotLib.util.InterpolatingTreeMap;
import frc.ExternalLib.PoofLib.util.InterpolatingDouble;
import frc.ExternalLib.PoofLib.util.InterpolatingTreeMap;
import frc.robot.Constants;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.Vision;


public class ShooterCommand extends CommandBase{
  
    private final ShooterSubsystem subsystem; 
    private final Vision Blindight; 
    private double speed;
    private Timer timer;


    public ShooterCommand(ShooterSubsystem shooter, Vision blindlight){
        this.subsystem = shooter;
        this.Blindight = blindlight;
      

        addRequirements(shooter);

    }
    
    @Override
    public void initialize() {
        subsystem.setFlywheelCurrentLimitEnabled(false);
 
    }
    @Override
    public void execute() {
           subsystem.RunShooter(ShooterConstants.ShooterVelocityTable.getInterpolated(new InterpolatingDouble(Blindight.getAvgDistance())).value);
        subsystem.setHoodTargetAngle((ShooterConstants.HoodPositionTable.getInterpolated(new InterpolatingDouble(Blindight.getAvgDistance()))).value);
  
           
    }

    @Override
    public void end(boolean interrupted) {
        subsystem.setFlywheelCurrentLimitEnabled(true);
        subsystem.stopFlywheel();
       
    }
}
    



    

