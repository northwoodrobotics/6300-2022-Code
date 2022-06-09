package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.FeederConstants;
import frc.ExternalLib.JackInTheBotLib.robot.UpdateManager;
import frc.ExternalLib.PicoPi.PicoColorSensor;
import frc.ExternalLib.PicoPi.PicoColorSensor.RawColor;



public class FeederSubsystem extends SubsystemBase {
    
    private TalonFX FeederMotor = new TalonFX(Constants.FeederConstants.FeederMotorID);
    private TalonFX RejectMotor = new TalonFX(FeederConstants.RejectMotorID);
    private PicoColorSensor BallSensor = new PicoColorSensor();
    private AnalogInput FeederStage2Sensor = new AnalogInput(Constants.FeederConstants.FeederStage2Sensor);
    private RawColor sensorColors = new RawColor();
    public AllianceColor feederMode; 
    private final NetworkTableEntry ShouldAdvance;
    private final NetworkTableEntry Stage2Loaded;
    private final NetworkTableEntry motorSpeed;
    private final NetworkTableEntry EjectorHasBall;
    private FeedMode feedMode;
    
    public FeederSubsystem(){
        feedMode = FeedMode.IDLE;
        FeederMotor.setInverted(false); 
        FeederMotor.setNeutralMode(NeutralMode.Brake);// set to break, so it holds the balls, and stops them at the right positions
        TalonFXConfiguration FeederConfig = new TalonFXConfiguration();
        FeederMotor.configAllSettings(FeederConfig);
        RejectMotor.configAllSettings(FeederConfig);
        FeederConfig.supplyCurrLimit.currentLimit = 20;
        ShutTalonUP(FeederMotor);
        ShutTalonUP(RejectMotor);
        EnableCurrentLimit();
        sensorColors = BallSensor.getRawColor0();
        ShuffleboardTab tab =Shuffleboard.getTab("Feeder");
        EjectorHasBall = tab.add("EjectorHasBall", false)
        .withPosition(0, 0).withSize(1, 1).getEntry();
        Stage2Loaded = tab.add("BallAtTurret", false)
        .withPosition(0, 1).withSize(1, 1).getEntry();
        motorSpeed = tab.add("MotorSpeed", 0.0)
        .withPosition(1, 0).withSize(1, 1).getEntry();
        ShouldAdvance = tab.add("ShouldAdvance", false).withPosition(1, 2).withSize(1, 1).getEntry();
    }
    public void SetIndex(){
        feedMode = FeedMode.INDEX;
    }
    public void SetFeed(){
        feedMode = FeedMode.FEED;
    }
    public void SetIdle(){
        feedMode = FeedMode.IDLE;
    }

    public void EnableCurrentLimit(){
        SupplyCurrentLimitConfiguration config = new SupplyCurrentLimitConfiguration();
        config.currentLimit = Constants.ShooterConstants.ShooterCurrentLimit;
        config.enable = true;
        FeederMotor.configSupplyCurrentLimit(config, 0);
        RejectMotor.configSupplyCurrentLimit(config, 0);

    }


    public void runFeeder(double speed){
        FeederMotor.set(ControlMode.PercentOutput, speed);// runs the feeder at set speeds, tuned during the season over and over again. 
    }

    public boolean Stage2Loaded(){
        return FeederStage2Sensor.getVoltage() <0.1; // a true or false value, when the sensor is broken
    }

    public boolean BallAtEjector(){
        if(BallSensor.getProximity1()>1){
            return true;
        }else return false;
    }

    public void SetFeederMode(){
        if(BallSensor.isSensor0Connected() && DriverStation.isEnabled()){
            switch (DriverStation.getAlliance()){
                case Red: 
                feederMode = AllianceColor.RED;
                break;
                case Blue: 
                feederMode = AllianceColor.BLUE;
                break;
                default:
                break;
            }
        }
    }
    public void SortBalls(){
        switch (feederMode){
            case RED:
            if(sensorColors.red<450 || sensorColors.green > sensorColors.red*2 && sensorColors.blue > 50){
                RejectMotor.set(ControlMode.PercentOutput, 1);
            }else if (sensorColors.red >130 ){
                if(shouldAdvance()){
                    FeederMotor.set(TalonFXControlMode.PercentOutput, 1);
                }else FeederMotor.set(TalonFXControlMode.PercentOutput, 0);
            }
            case BLUE:
            if(sensorColors.red >130){
                RejectMotor.set(ControlMode.PercentOutput, 1);
            }else if (sensorColors.red<450 || sensorColors.green > sensorColors.red*2 && sensorColors.blue > 50){
                if(shouldAdvance()){
                    FeederMotor.set(TalonFXControlMode.PercentOutput, 1);
                }else FeederMotor.set(TalonFXControlMode.PercentOutput, 0);
            }
        }
    }
  
    public boolean shouldAdvance(){
        if (BallAtEjector()){
            if(Stage2Loaded()){
                return false;
            }else return true;
        }
       return true;
    }


    public void ShutTalonUP(TalonFX targetTalon){
        targetTalon.setStatusFramePeriod(4, 251);
        targetTalon.setStatusFramePeriod(10,251);
        targetTalon.setStatusFramePeriod(12,251);
        targetTalon.setStatusFramePeriod(13,251);
        targetTalon.setStatusFramePeriod(21,251);
        targetTalon.setStatusFramePeriod(4, 251);
    }




    
   

    @Override
    public void periodic() {
        Stage2Loaded.setBoolean(Stage2Loaded());
        EjectorHasBall.setBoolean(BallAtEjector());
        ShouldAdvance.setBoolean(shouldAdvance());
        motorSpeed.setDouble(FeederMotor.getSelectedSensorVelocity());
        switch (feedMode){
            case IDLE: 
            FeederMotor.set(ControlMode.Disabled, 0);
            RejectMotor.set(ControlMode.Disabled, 0);
            break; 
            case INDEX:
            SortBalls();
            break; 
            case FEED: 
            FeederMotor.set(ControlMode.PercentOutput, 1);
            break; 
        }  
    }

    public enum AllianceColor{
        RED,BLUE
    }
    public enum FeedMode{
        INDEX, FEED, IDLE
    }
    public int redBall;
    public int blueBall;
    

    



    
}
