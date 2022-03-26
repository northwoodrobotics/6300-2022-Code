package frc.robot.commands.SubsystemCommands.AutoCommands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ShooterConstants;
//import frc.ExternalLib.CitrusLib.CitrusConstants.ShooterConstants;
import frc.ExternalLib.JackInTheBotLib.util.InterpolatingDouble;
import frc.ExternalLib.JackInTheBotLib.util.InterpolatingTreeMap;
import frc.robot.Constants;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.Vision;


public class AutoShoot extends CommandBase{
    //private static final InterpolatingTreeMap<InterpolatingDouble, Translation2d> ShooterTuning = new InterpolatingTreeMap<>();

    private final ShooterSubsystem subsystem; 
    private final Vision Blindight; 
    private double speed;
    private Timer timer;


    public AutoShoot(ShooterSubsystem shooter, Vision blindlight){
        this.subsystem = shooter;
        //this.speed = speed;
        this.Blindight = blindlight;
        timer = new Timer();

        addRequirements(shooter);

    }
    
    @Override
    public void initialize() {
        subsystem.setFlywheelCurrentLimitEnabled(false);
        timer.start();
 
    }
    @Override
    public void execute() {
        //timer.start();
        //subsystem.percentoutput(1);   
        
        //subsystem.RunShooter(Constants.ShooterConstants.ShooterVelocityTable.lookup(Blindight.getRobotToTargetDistance()));
        subsystem.RunShooter(-9000);
        //subsystem.MoveHood((Constants.ShooterConstants.HoodPositionTable.lookup(Math.round(Blindight.getAvgDistance() *10/10))));
        //subsystem.setHoodTargetAngle((Constants.ShooterConstants.HoodPositionTable.lookup(Math.round(Blindight.getAvgDistance() *10/10))));  
           
    }

    @Override
    public void end(boolean interrupted) {
        subsystem.setFlywheelCurrentLimitEnabled(true);
        subsystem.stopFlywheel();
        
    }
    @Override 
    public boolean isFinished(){
        return timer.get()>4.0;
    }
}
    



    

