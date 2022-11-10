package frc.robot.commands.DriveCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Vision;
import frc.swervelib.SwerveSubsystem;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class PoseFromVision extends CommandBase{
    private final SwerveSubsystem m_SwerveSubsystem;
    private final Vision visionsystem;
    private Timer posetimer;
    public PoseFromVision(SwerveSubsystem subsystem, Vision vision){
        m_SwerveSubsystem = subsystem;
        visionsystem = vision;
        posetimer = new Timer();
        addRequirements(subsystem, vision);
    }
    @Override
    public void initialize(){
        posetimer.reset();
        posetimer.start();
       
       SmartDashboard.getNumber("PreVisonPoseX", m_SwerveSubsystem.dt.getPose().getX());
       SmartDashboard.getNumber("PreVisonPoseY", m_SwerveSubsystem.dt.getPose().getY());
       SmartDashboard.getNumber("PreVisonPoseRot", m_SwerveSubsystem.dt.getPose().getRotation().getDegrees());
    }
    @Override
    public void execute(){
        if (visionsystem.hasTarget()&& visionsystem.getTargetAngleX() <0.1){
            m_SwerveSubsystem.dt.VisionPose(PoseFromVison(visionsystem.getRobotToTargetDistance()));

        }
    }
    @Override 
    public void end(boolean interrupted){
        SmartDashboard.getNumber("PostVisonPoseX", m_SwerveSubsystem.dt.getPose().getX());
       SmartDashboard.getNumber("PostVisonPoseY", m_SwerveSubsystem.dt.getPose().getY());
       SmartDashboard.getNumber("PostVisonPoseRot", m_SwerveSubsystem.dt.getPose().getRotation().getDegrees());
    }
    @Override
    public boolean isFinished(){
        return posetimer.hasElapsed(.25);
    }



    
    public Pose2d PoseFromVison(double distance){
        Pose2d CalculatedPose; 
    CalculatedPose  = Constants.VisionConstants.GoalPose.plus(new Transform2d(new Translation2d(distance, RobotContainer.dt.getGyroscopeRotation()), RobotContainer.dt.getGyroscopeRotation()));
        return CalculatedPose;

    
        

}
}
