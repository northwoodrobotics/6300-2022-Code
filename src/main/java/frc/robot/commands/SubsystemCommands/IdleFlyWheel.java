package frc.robot.commands.SubsystemCommands;

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


public class IdleFlyWheel extends CommandBase{
    //private static final InterpolatingTreeMap<InterpolatingDouble, Translation2d> ShooterTuning = new InterpolatingTreeMap<>();

    private final ShooterSubsystem subsystem; 
    private final Vision Blindight; 


    public IdleFlyWheel(ShooterSubsystem shooter, Vision blindlight){
        this.subsystem = shooter;
        //this.speed = speed;
        this.Blindight = blindlight;
      

        addRequirements(shooter);

    }
    
    @Override
    public void initialize() {
        subsystem.setFlywheelCurrentLimitEnabled(false);
        
 
    }
    @Override
    public void execute() {
        subsystem.RunShooter(-4000);
        
           
    }

    @Override
    public void end(boolean interrupted) {
        subsystem.setFlywheelCurrentLimitEnabled(true);
        subsystem.stopFlywheel();
       
    }
}
    



    

