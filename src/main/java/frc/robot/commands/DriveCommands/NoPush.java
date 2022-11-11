package frc.robot.commands.DriveCommands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.swervelib.SwerveSubsystem;

public class NoPush extends CommandBase{
    private final SwerveSubsystem m_SwerveSubsystem;
    private  SwerveModuleState[] nopushStates;
    
    
    public NoPush(SwerveSubsystem subsystem){
        this.m_SwerveSubsystem = subsystem;
        addRequirements(subsystem);
    }
    public void setNopushStates(){
      nopushStates[0]= new SwerveModuleState(0, Rotation2d.fromDegrees(45+90));
      nopushStates[1]= new SwerveModuleState(0, Rotation2d.fromDegrees(315+90));
      nopushStates[2]= new SwerveModuleState(0, Rotation2d.fromDegrees(225+90));
      nopushStates[3]= new SwerveModuleState(0, Rotation2d.fromDegrees(135+90));
    }
    @Override
    public void initialize(){
      setNopushStates();
    }


    @Override
    public void execute(){

      m_SwerveSubsystem.dt.setModuleStates(
          nopushStates
          )    
        ;
    
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
