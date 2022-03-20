package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.ExternalLib.JackInTheBotLib.robot.UpdateManager;


public class FeederSubsystem extends SubsystemBase {
    private TalonFX FeederMotor = new TalonFX(Constants.FeederConstants.FeederMotorID);
    //private AnalogInput FeederStage1Sensor = new AnalogInput(Constants.FeederConstants.FeederStage1Sensor);
    private AnalogInput FeederStage2Sensor = new AnalogInput(Constants.FeederConstants.FeederStage2Sensor);
    private AnalogInput IntakeSensor = new AnalogInput(Constants.FeederConstants.IntakeSensor);
    //\private Timer AdvanceTimer = new Timer();
    //private RelativeEncoder FeederEncoder;
    private final NetworkTableEntry Stage1Loaded;
    private final NetworkTableEntry ShouldAdvance;
    private final NetworkTableEntry Stage2Loaded;
    private final NetworkTableEntry motorSpeed;
    private final NetworkTableEntry IntakeHasBall;


    public FeederSubsystem(){
        FeederMotor.setInverted(false);
        FeederMotor.setNeutralMode(NeutralMode.Brake);
        TalonFXConfiguration FeederConfig = new TalonFXConfiguration();
        //FeederConfig.pr
        FeederMotor.setStatusFramePeriod(3, 150);

        ShuffleboardTab tab =Shuffleboard.getTab("Feeder");
        Stage1Loaded = tab.add("Do We Have 1 Ball", false)
        .withPosition(0, 0).withSize(1, 1).getEntry();
        Stage2Loaded = tab.add("Do We Have 2 Balls", false)
        .withPosition(0, 1).withSize(1, 1).getEntry();
        IntakeHasBall = tab.add("Does The Intake Have a Ball", false)
        .withPosition(0, 3).withSize(1, 1).getEntry();
        motorSpeed = tab.add("MotorSpeed", 0.0)
        .withPosition(1, 0).withSize(1, 1).getEntry();
        ShouldAdvance = tab.add("ShouldAdvance", false).withPosition(1, 2).withSize(1, 1).getEntry();
    }
    public void runFeeder(double speed){
        FeederMotor.set(ControlMode.PercentOutput, speed);
    }

    /*public boolean Stage1Loaded(){
        return FeederStage1Sensor.getVoltage() < 0.1;

    }*/
    public boolean Stage2Loaded(){
        return FeederStage2Sensor.getVoltage() <0.1;
    }
    public boolean IntakeHasBall(){
        return IntakeSensor.getVoltage() <0.1;
    }

    public boolean shouldAdvance(){
        if(Stage2Loaded()){
            return false;
        }
        return IntakeHasBall();
    }

    
   

    @Override
    public void periodic() {
        //Stage1Loaded.setBoolean(Stage1Loaded());
        Stage2Loaded.setBoolean(Stage2Loaded());
        IntakeHasBall.setBoolean(IntakeHasBall());
        ShouldAdvance.setBoolean(shouldAdvance());
            motorSpeed.setDouble(FeederMotor.getSelectedSensorVelocity());


        
    }

    

    



    
}
