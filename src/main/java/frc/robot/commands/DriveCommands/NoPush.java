package frc.robot.commands.DriveCommands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.swervelib.SwerveSubsystem;

public class NoPush extends CommandBase{
    private final SwerveSubsystem m_SwerveSubsystem;
    
    
    public NoPush(SwerveSubsystem subsystem){
        this.m_SwerveSubsystem = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void execute(){
     m_SwerveSubsystem.dt.setNoPush();
    }
    @Override
    public void end(boolean interrupted) {
        m_SwerveSubsystem.dt.setModuleStates(
          Constants.DriveConstants.KINEMATICS.toSwerveModuleStates(  
            new ChassisSpeeds(0.0, 0.0, 0.0)
          )    
        );


    }


}
