package frc.robot.commands.DriveCommands;

import org.photonvision.common.hardware.VisionLEDMode;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.PhotonCams;
import frc.swervelib.SwerveSubsystem;

public class DriveToBall extends CommandBase{
    private final SwerveSubsystem m_SwerveSubsystem;
    private final PhotonCams Camera;
    private Timer DriveTimer = new Timer();
    private Pose2d CurrentPose;
    private Pose2d BallPose;
    private Transform2d BallRelative;
   
    public DriveToBall(SwerveSubsystem subsystem, PhotonCams camera){
        this.m_SwerveSubsystem = subsystem;
        this.Camera = camera;

    }
    @Override
    public void initialize(){

        DriveTimer.reset();
        
       
       Camera.setIntakeCam(VisionLEDMode.kOn);
       CurrentPose= m_SwerveSubsystem.dt.getPose();
       BallRelative = Camera.BallLocation();
       BallPose = CurrentPose.transformBy(BallRelative);
       
    }
    @Override
    public void execute(){
        DriveTimer.start();
        m_SwerveSubsystem.dt.goToPose(BallPose, CurrentPose.getRotation().getDegrees()-BallRelative.getRotation().getDegrees());

    }
    @Override
    public void end(boolean interrupted){
        
        SmartDashboard.putNumber("DriveTime", DriveTimer.get());
        m_SwerveSubsystem.dt.driveClean(0, 0, 0);
    }




    
    
}
