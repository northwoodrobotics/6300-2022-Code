package frc.ExternalLib.NorthwoodLib.NorthwoodDrivers;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.ExternalLib.BionicsLib.IYawEncoder;

public class RevThroughBore implements IYawEncoder{
    private DutyCycleEncoder  encoder;

    private double encoderOffset;
    private LinearFilter yawDistanceAverage;
    private double offset;


    
    private String name;
    private NetworkTableEntry sb_angle;
    private NetworkTableEntry sb_angleOffset;



    public RevThroughBore(int dioChannel, String name, double offset){
        this.name =name; 
        this.encoderOffset = offset;
       
        this.yawDistanceAverage = LinearFilter.movingAverage(10);
        encoder = new DutyCycleEncoder(dioChannel);
        encoder.setDistancePerRotation(360.0);
        

    }
    public double getDistanceDegrees()
    {
      double currentYawDist = this.yawDistanceAverage.calculate(encoder.getDistance());
      return frc.ExternalLib.BionicsLib.Conversion.normalize(currentYawDist + encoderOffset, -180, 180);
    }
    

    
    
}
