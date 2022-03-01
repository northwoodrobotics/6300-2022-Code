package frc.robot.subsystems;

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
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ClimberConstants;

public class ClimberSubsystem extends SubsystemBase{
    //private TalonSRX TestClimbMotor = new TalonSRX(31);
    //real stuff
    private CANSparkMax ClimbMotor1 = new CANSparkMax(Constants.ClimberConstants.ClimbMotor1, MotorType.kBrushless);
    private CANSparkMax ClimbMotor2 = new CANSparkMax(Constants.ClimberConstants.ClimbMotor2, MotorType.kBrushless);
    private DoubleSolenoid lockSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, ClimberConstants.ClimbSolenoid,ClimberConstants.ClimbSolenoid2);
    private DoubleSolenoid BalanceSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, ClimberConstants.BalanceSolenoid, ClimberConstants.BalanceSolenoid2);
    //private Solenoid CatchSolenoid = new Solenoid(PneumaticsModuleType.REVPH, ClimberConstants.CatchSolenoid);
    private SparkMaxPIDController Climb1Controller;
    private SparkMaxPIDController Climb2Controller;
    private RelativeEncoder Climb1Encoder;
    private RelativeEncoder Climb2Encoder;
    //private final Object stateLock = new Object();

    //@GuardedBy("stateLock")
    //private boolean LatchOn = false;
    //@GuardedBy("stateLock")
    private boolean ClimbUp = false;
    // fix me values
    private double Climb1Out = 0;
    private double Climb2Out = 0; 
    private double Climb1Release = 0;

    

    public ClimberSubsystem(){
        setBreakModes();
        Climb1Controller = ClimbMotor1.getPIDController();
        Climb1Controller.setP(ClimberConstants.Climb1P);
        Climb1Controller.setI(ClimberConstants.Climb1I);
        Climb1Controller.setD(ClimberConstants.Climb1D);
        Climb1Controller.setOutputRange(-1, 1);
        




        Climb2Controller = ClimbMotor2.getPIDController();
        Climb1Encoder = ClimbMotor1.getEncoder();
        Climb1Encoder.setPositionConversionFactor(ClimberConstants.Climb1GearRatio);
        Climb2Encoder.setPositionConversionFactor(ClimberConstants.Climb2GearRatio);
        Climb2Encoder = ClimbMotor2.getEncoder();
        Climb2Controller.setP(ClimberConstants.Climb2P);
        Climb2Controller.setI(ClimberConstants.Climb2I);
        Climb2Controller.setD(ClimberConstants.Climb2D);
        Climb2Controller.setOutputRange(-1, 1);


        
    }
    
    public void setBreakModes(){
        ClimbMotor1.setIdleMode(IdleMode.kBrake);
        ClimbMotor2.setIdleMode(IdleMode.kBrake);
    }
    public void setCurrentLimits(){
        ClimbMotor1.enableSoftLimit(SoftLimitDirection.kForward, true);
        ClimbMotor2.enableSoftLimit(SoftLimitDirection.kForward, true);
        ClimbMotor1.setSoftLimit(SoftLimitDirection.kForward, ClimberConstants.Climb1SoftForward);
        ClimbMotor1.setSoftLimit(SoftLimitDirection.kReverse, ClimberConstants.Climb1SoftReverse);
        ClimbMotor2.setSoftLimit(SoftLimitDirection.kForward, ClimberConstants.Climb2SoftForward);
        ClimbMotor2.setSoftLimit(SoftLimitDirection.kReverse, ClimberConstants.Climb2SoftReverse);


    }


    public void Climb1ToPositoin(double setpoint){
        Climb1Controller.setReference(setpoint, ControlType.kPosition);
    }

    public void Climb2ToPosition(double setpoint){
        Climb2Controller.setReference(setpoint, ControlType.kPosition);
    }




    private enum climbstate{
        Stowed,
        StartClimb,
        Climb,ToNextBar
        ,lock
    }
    public climbstate state = climbstate.Stowed;

    @Override 
    public void periodic(){
        switch (state){
            case Stowed:
             Climb1ToPositoin(0);
             Climb2ToPosition(0);
             BalanceSolenoid.set(Value.kReverse);
             lockClimb();
             
             break;
             case StartClimb: 
             Climb1ToPositoin(Climb1Out);
             Climb2ToPosition(Climb2Out);
             BalanceSolenoid.set(Value.kForward);
             lockSolenoid.set(Value.kReverse);
             break; 
             case Climb:
             Climb1ToPositoin(0);
             if( getClimb1Position() != 0) {
                
                state = climbstate.ToNextBar;
             } 
             break; 
             case ToNextBar: {
                 Climb2ToPosition(0);
                 if(Climb1Encoder.getVelocity()> 0){
                     Climb1ToPositoin(Climb1Release);
                 }
                 if(Climb2Encoder.getPosition()!= 0){
                     state = climbstate.Stowed;
                 }
             }
            
             }
        }
        
    

    public void lockClimb(){
        if (Climb2Encoder.getVelocity()>0){
            lockSolenoid.set(Value.kReverse);
        }else {
            lockSolenoid.set(Value.kForward);
        }
    }
    public double getClimb1Position(){
        return Climb1Encoder.getPosition();
    }



   



    
    
}
