package frc.robot.commands.SubsystemCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;

public class HomeHood extends CommandBase{
    private ShooterSubsystem shooter;
    private long zeroVelocityTimestamp;
    private final long HOOD_ZERO_VELOCITY_TIME_PERIOD = 250;//in ms

    public HomeHood(ShooterSubsystem subsystem){
        this.shooter = subsystem;
        addRequirements(subsystem);
    }
    /*
    @Override
    public void initialize() {
        shooter.setHoodHomed(false);
        shooter.setHoodMotorPower(0.1);
        zeroVelocityTimestamp = System.currentTimeMillis();
    }*/
    public void execute(){
      //  shooter.setHoodHomed(true);
    }
    /*
    @Override
    public boolean isFinished() {

        if(Math.abs(shooter.getHoodVelocity()*266.67) > Math.toRadians(0.01)){
            zeroVelocityTimestamp = System.currentTimeMillis();
        }

        if(System.currentTimeMillis() - zeroVelocityTimestamp >= HOOD_ZERO_VELOCITY_TIME_PERIOD){
            shooter.setHoodHomed(true);
            return true;
        }

        return false;
    }*/
}