package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;


import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Vision extends SubsystemBase{
    NetworkTable limelight;

    public enum LEDMode {
		PIPELINE(0),
		LED_OFF(1),
		LED_BLINK(2),
		LED_ON(3);
		
		private int m;
		
		LEDMode(int mode) {
			m = mode;
		}
		
		public int getMode() {
			return m;
		}


	}
    public enum StreamingMode {
		STANDARD(0),
		PIP_MAIN(1),
		PIP_SECONDARY(2);
		
		private int m;
		
		StreamingMode(int mode) {
			m = mode;
		}
		
		public int getMode() {
			return m;
		}
	}

    public enum CameraMode {
		VISION_PROCESSING(0),
		DRIVER_CAMERA(1);
		
		private int m;
		
		CameraMode(int mode) {
			m = mode;
		}
		
		public int getMode() {
			return m;
		}
	}
    
	public Vision() {
		limelight = NetworkTableInstance.getDefault().getTable("limelight-front");

		setLEDMode(LEDMode.LED_OFF);
		setStreamingMode(StreamingMode.STANDARD);
	}
    public boolean hasTarget() {
		return limelight.getEntry("tv").getDouble(0) == 1;
	}



	




    
	public void setLEDMode(LEDMode mode) {
		limelight.getEntry("ledMode").setNumber(mode.getMode());
	}
	
	/**
	 * Set the camera mode
	 * @param mode The mode to use
	 */
	public void setCameraMode(CameraMode mode) {
		limelight.getEntry("camMode").setNumber(mode.getMode());
	}
	
	/**
	 * Set the streaming mode
	 * @param mode The mode to use
	 */
	public void setStreamingMode(StreamingMode mode) {
		limelight.getEntry("stream").setNumber(mode.getMode());
	}
	
	/**
	 * Set the processing pipeline
	 * @param id The id of the pipeline to use
	 */
	public void setPipeline(int id) {
		limelight.getEntry("pipeline").setNumber(id);
	}













    
    







    
}
