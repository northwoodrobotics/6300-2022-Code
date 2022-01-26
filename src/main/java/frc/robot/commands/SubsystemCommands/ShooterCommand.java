package frc.robot.commands.SubsystemCommands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.ExternalLib.JackInTheBotLib.util.InterpolatingDouble;
import frc.ExternalLib.JackInTheBotLib.util.InterpolatingTreeMap;
import frc.robot.Constants;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.Vision;


public class ShooterCommand extends CommandBase{
    //private static final InterpolatingTreeMap<InterpolatingDouble, Translation2d> ShooterTuning = new InterpolatingTreeMap<>();

    private final ShooterSubsystem subsystem; 
    private final Vision Blindight; 


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
        subsystem.RunShooter(Constants.ShooterConstants.ShooterVelocityTable.lookup(Blindight.getRobotToTargetDistance()));
        subsystem.setHoodTargetAngle(Constants.ShooterConstants.HoodPositionTable.lookup(Blindight.getRobotToTargetDistance()));
    }
    @Override
    public void end(boolean interrupted) {
        subsystem.setFlywheelCurrentLimitEnabled(true);
        subsystem.stopFlywheel();
       
    }
}
    



    

