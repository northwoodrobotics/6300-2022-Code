package frc.robot.commands.DriveCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.PhotonCams;
import frc.swervelib.SwerveSubsystem;

public class DriveToBall extends CommandBase{
    private final SwerveSubsystem m_SwerveSubsystem;
    private final PhotonCams Camera;
    private Timer DriveTimer;
    private Pose2d CurrentPose;
    private Pose2d BallPose;
    private Transform2d BallRelative;

    public DriveToBall(SwerveSubsystem subsystem, PhotonCams camera){
        this.m_SwerveSubsystem = subsystem;
        this.Camera = camera;

    }
    @Override
    public void initialize(){
       CurrentPose= m_SwerveSubsystem.dt.getPose();
       BallRelative = Camera.BallLocation();
       BallPose = CurrentPose.transformBy(BallRelative);
       
    }
    @Override
    public void execute(){
        m_SwerveSubsystem.dt.goToPose(BallPose, BallRelative.getRotation().getDegrees());

    }
    @Override
    public void end(boolean interrupted){
        m_SwerveSubsystem.dt.driveClean(0, 0, 0);
    }




    
    
}
