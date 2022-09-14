package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;


import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class ClimberSubsystem extends SubsystemBase{
   
    //real stuff
    private TalonFX Climb1Talon = new TalonFX(ClimberConstants.ClimbMotor1);
    private TalonFX Climb2Talon = new TalonFX(ClimberConstants.ClimbMotor2);
   
    public ClimberSubsystem(){
        
       
        TalonFXConfiguration Climb1Config = new TalonFXConfiguration();
        Climb1Config.slot0.kP = ClimberConstants.Climb1P;
        Climb1Config.slot0.kI = ClimberConstants.Climb1I;
        Climb1Config.slot0.kD = ClimberConstants.Climb1D;
        Climb1Config.slot0.kF = ClimberConstants.Climb1F;
        Climb1Config.motionAcceleration = ClimberConstants.Climb1MotionAccel; 
        Climb1Config.motionCruiseVelocity = ClimberConstants.Climb1MotionVelocity;
        Climb1Config.motionCurveStrength = 1; 
        Climb1Config.primaryPID.selectedFeedbackSensor = TalonFXFeedbackDevice.IntegratedSensor.toFeedbackDevice();
        Climb1Config.supplyCurrLimit.currentLimit= 30;
        Climb1Config.supplyCurrLimit.enable = true; 
        Climb1Talon.configAllSettings(Climb1Config);
        Climb1Talon.setNeutralMode(NeutralMode.Brake);
        Climb1Talon.setStatusFramePeriod(3, 200);
        Climb2Talon.setStatusFramePeriod(3, 200);
        // fix me 
        Climb1Talon.setInverted(false);
        TalonFXConfiguration Climb2Config = new TalonFXConfiguration();
        Climb2Config.slot0.kP = ClimberConstants.Climb2P;
        Climb2Config.slot0.kI = ClimberConstants.Climb2I;
        Climb2Config.slot0.kD = ClimberConstants.Climb2D;
        Climb2Config.slot0.kF = ClimberConstants.Climb2F;
        Climb2Config.motionAcceleration = ClimberConstants.Climb2MotionAccel;
        Climb2Config.motionCruiseVelocity = ClimberConstants.Climb2MotionVelocity;
        Climb2Config.motionCurveStrength = 1;
        Climb2Config.primaryPID.selectedFeedbackSensor = TalonFXFeedbackDevice.IntegratedSensor.toFeedbackDevice();
        Climb2Config.supplyCurrLimit.currentLimit= 30;
        Climb2Config.supplyCurrLimit.enable = true; 
        Climb2Talon.configAllSettings(Climb1Config);
        Climb2Talon.setNeutralMode(NeutralMode.Brake);

        
        Climb2Talon.setInverted(false);
        Climb1Talon.setInverted(false);


       


        
    }
    
   
    public void ExtendClimb(){
        Climb1Talon.set(TalonFXControlMode.PercentOutput, 1);
        Climb2Talon.set(TalonFXControlMode.PercentOutput, 1);
    }
    public void Climb(){
        Climb1Talon.set(TalonFXControlMode.PercentOutput, -1);
        Climb2Talon.set(TalonFXControlMode.PercentOutput, -1);
    }
    public void HoldClimb(){
        Climb1Talon.set(TalonFXControlMode.Disabled, 0);
        Climb2Talon.set(TalonFXControlMode.Disabled, 0);
        
    }

    public void Climb1ToPositoin(double setpoint){
        Climb1Talon.set(ControlMode.MotionMagic, setpoint);
           
    }

    public void Climb2ToPosition(double setpoint){
        Climb2Talon.set(ControlMode.MotionMagic, setpoint);
        
    }

    private enum climbstate{
        Stowed,
        StartClimb,ClimbMid,WaitForSwing
        ,
        ToHighBar,ToTraverse
        ,UnlatchHighBar, PercentOutput
    }
    public climbstate state = climbstate.Stowed;

    @Override 
    public void periodic(){
    
     // State Machine for Citrus Climber 
        switch (state){
             case Stowed:
             Climb1ToPositoin(0);
             Climb2ToPosition(0);
             break;
             case StartClimb:
             Climb1ToPositoin(20000);
             break; 
             case ClimbMid: 
             Climb1ToPositoin(0);
             Climb2ToPosition(15000);
             break;
             case WaitForSwing:
             Climb2ToPosition(20000);
             break;
             case ToHighBar: 
             Climb2ToPosition(0);
             Climb1ToPositoin(15000);
             break; 
             case ToTraverse: 
             Climb2ToPosition(0);
             Climb1ToPositoin(20000);
             break; 
             case UnlatchHighBar:
             Climb2ToPosition(4000);
             break;
                  
             }
    }

    public void StartClimb(){
        state = climbstate.StartClimb;
    }


       
        
    

    public void GoClimb(){

        state = climbstate.WaitForSwing;
       
        
        
    }
    public void Transition1(){

        state = climbstate.ToHighBar;
       
        
        
    }

    public void Transistion2(){
        state = climbstate.ToTraverse;
    } 

    public void Unhook(){
        state = climbstate.UnlatchHighBar; 
    }
    


    public boolean IsHook1Climbing(){
        if(Climb1Talon.getStatorCurrent()> 20){
            return true; 
        }else return false; 
    }
    
    public boolean IsHook2Climbing(){
        if(Climb2Talon.getStatorCurrent()> 20){
            return true; 
        }else return false; 
    }
    
 



   



    
    
}
