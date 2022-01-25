package frc.ExternalLib.NorthwoodLib.NorthwoodDrivers;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.ExternalLib.JackInTheBotLib.math.MathUtils;
import edu.wpi.first.math.util.Units;

public class HuskyLimeLight {
    // written by Kirk Nguyenle

    //create limelight network table 
    private final NetworkTable table;
    //create target Aqu entry
    private final NetworkTableEntry TargetAqu;

    // create x offset from crosshair entry
    private final NetworkTableEntry XOffset; 

    //create y offset 
    private final NetworkTableEntry YOffset; 
    private final NetworkTableEntry TargetArea;
    private final NetworkTableEntry TargetDegreeOffset;
    private final NetworkTableEntry Latency;

    
    private final NetworkTableEntry ledMode;
    private final NetworkTableEntry camMode;
    private final NetworkTableEntry pipeline;
    private final NetworkTableEntry stream;
    private final NetworkTableEntry snapshot;



    



    public HuskyLimeLight(){
        this(NetworkTableInstance.getDefault().getTable("limelight"));
    }


    public HuskyLimeLight(NetworkTable table){
        this.table = table;


        TargetAqu = table.getEntry("tv");


        XOffset = table.getEntry("tx");
        YOffset = table.getEntry("ty");
        TargetArea = table.getEntry("ta");
        TargetDegreeOffset = table.getEntry("ts");
        Latency = table.getEntry("tl");


        ledMode = table.getEntry("ledMode");
        camMode = table.getEntry("camMode");
        pipeline = table.getEntry("pipeline");
        stream = table.getEntry("stream");
        snapshot = table.getEntry("snapshot");
    
    }

    public boolean hasTarget() {
        return MathUtils.epsilonEquals(TargetArea.getDouble(0), 1);

    }








       
    
















}
