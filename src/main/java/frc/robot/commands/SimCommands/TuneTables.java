package frc.robot.commands.SimCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.SubsystemCommands.ShooterCommand;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.Vision;

public class TuneTables extends CommandBase{
    private final ShooterSubsystem subsystem; 
    //private final Vision Blindight; 
    private double m_speed;
    
    


    public TuneTables(double speed, ShooterSubsystem shooter){
       
        this.m_speed = speed;
        this.subsystem = shooter;
    }

    @Override
    public void initialize() {
       // subsystem.setFlywheelCurrentLimitEnabled(false);
    }
        
    @Override
    public void execute() {
        //subsystem.percentoutput(1);   
        //subsystem.RunShooter(Constants.ShooterConstants.ShooterVelocityTable.lookup(Blindight.getRobotToTargetDistance()));
        subsystem.RunShooter(m_speed);
        //subsystem.MoveHood((Constants.ShooterConstants.HoodPositionTable.lookup(Math.round(Blindight.getAvgDistance() *10/10))));
        //subsystem.MoveHood(m_angle);     
    }
    @Override
    public void end(boolean interrupted) {
        //subsystem.setFlywheelCurrentLimitEnabled(true);
        //subsystem.stopFlywheel();
       
    }


}
