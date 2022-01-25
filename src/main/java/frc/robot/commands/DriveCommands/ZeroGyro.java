package frc.robot.commands.DriveCommands;

import frc.swervelib.SwerveSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
public class ZeroGyro extends CommandBase{
    SwerveSubsystem m_swerveSubsystem;
    
   public ZeroGyro(SwerveSubsystem subsystem){
       this.m_swerveSubsystem = subsystem;
       

    }
    @Override
    public void execute(){
        m_swerveSubsystem.dt.zeroGyroscope();
    }
}
