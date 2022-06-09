package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.google.errorprone.annotations.concurrent.GuardedBy;
import com.revrobotics.CANSparkMax;
import com.revrobotics.EncoderType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAlternateEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ClimberConstants;

public class ClimberSubsystem extends SubsystemBase{
   
    //real stuff
    private TalonFX Climb1Talon = new TalonFX(ClimberConstants.ClimbMotor1);
    private TalonFX Climb2Talon = new TalonFX(ClimberConstants.ClimbMotor2);
   
   

    private boolean ClimbUp = false;
    // fix me values
    private double Climb1Out = 0;
    private double Climb2Out = 0; 
    private double Climb1Release = 0;

    

    public ClimberSubsystem(){
        
        setBreakModes();
        //Climb1Controller = ClimbMotor1.getPIDController();
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
    
    public void setBreakModes(){
        
        
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
        StartClimb,
        Climb,ToNextBar
        ,lock, PercentOutput
    }
    public climbstate state = climbstate.Stowed;

    @Override 
    public void periodic(){
        /*
         Theoretical Traversal climb State Machine. 
        switch (state){
            case Stowed:
             Climb1ToPositoin(0);
             Climb2ToPosition(0);
             //BalanceSolenoid.set(Value.kReverse);
             lockClimb();
             
             break;
             case StartClimb: 
             Climb1ToPositoin(Climb1Out);
             BalanceServo.setPosition(1.8);
             
            // BalanceSolenoid.set(Value.kForward);
            // lockSolenoid.set(Value.kReverse);
            // if (lockSolenoid.get() != Value.kReverse){
                Climb2ToPosition(Climb2Out);
             
             break; 
             case Climb:
             Climb1ToPositoin(0);
             if (Climb1Talon.getSelectedSensorPosition()< 300){
                state = climbstate.ToNextBar;
             }
          
             break; 
             case ToNextBar: {
                 Climb2ToPosition(0);
                 if (IsHook2Climbing() != false){
                    Climb1ToPositoin(Climb1Release);
                 }
                 if(Climb2Talon.getSelectedSensorPosition() < 300){
                     state = climbstate.lock;
                 }
                 /*if(Climb1Encoder.getVelocity()> 0){
                     Climb1ToPositoin(Climb1Release);
                 }
                 if(Climb2Encoder.getPosition()!= 0){
                     state = climbstate.Stowed;
                 }
             }break;
             case lock:{
                 if (IsHook1Climbing() != false ){
                     Climb1ToPositoin(0);
                 }
                 lockClimb();
                 //BalanceSolenoid.set(Value.kReverse);
                 Climb2ToPosition(0);
                
             }
             
             
            
             }*/
        }

        public void StartClimb(){
            state = climbstate.StartClimb;
        }


       
        
    

    public void lockClimb(){
       
            //lockSolenoid.set(Value.kForward);
        
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
