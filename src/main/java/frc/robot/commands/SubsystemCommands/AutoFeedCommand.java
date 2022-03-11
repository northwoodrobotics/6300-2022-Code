package frc.robot.commands.SubsystemCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.Vision;
import frc.swervelib.SwerveSubsystem;

public class AutoFeedCommand extends CommandBase{
    private SwerveSubsystem drive; 
    private FeederSubsystem feeder; 
    private ShooterSubsystem shooter; 
    private Vision vision;
    private Timer timer; 

    public AutoFeedCommand(SwerveSubsystem m_swerveSubsystem, FeederSubsystem feeder, ShooterSubsystem shooter, Vision blindlight){
        this.drive = m_swerveSubsystem;
        this.feeder = feeder; 
        this.shooter = shooter;
        this.vision = blindlight;
    }
    @Override
    public void initialize(){
        timer.stop();
        timer.reset();
    }
    @Override
    public void execute(){
        if(vision.isOnTarget()&& shooter.isFlyWheelAtTargetVelocity()&& shooter.isHoodAtTargetAngle()){
            timer.start();
            if(timer.hasElapsed(0.05)){
                feeder.runFeeder(1);
            }else {
                feeder.runFeeder(0);
            }
            
        }
        else{
            feeder.runFeeder(0);
            timer.reset();
        }
    }
    @Override
    public void end(boolean interrupted) {
        feeder.runFeeder(0.0);
        //intakeSubsystem.setMotorOutput(0.0);
    }

    
    
}
