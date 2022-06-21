package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.common.hardware.VisionLEDMode;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PhotonCams extends SubsystemBase{
    PhotonCamera TurretCam = new PhotonCamera("TurretCam");
    PhotonCamera intakeCam = new PhotonCamera("intakeCam");
    public PhotonCams(){
        TurretCam.setDriverMode(false);
        TurretCam.setLED(VisionLEDMode.kOn);
        
        
    }

    public Transform2d BallLocation(){
        var res = intakeCam.getLatestResult();
        double imageCaptureTime = Timer.getFPGATimestamp() - res.getLatencyMillis();
        Transform2d camToBall = res.getBestTarget().getCameraToTarget();
        

      return camToBall;
        
    }




    




    
}
