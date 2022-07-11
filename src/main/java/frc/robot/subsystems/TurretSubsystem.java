package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.TurretConstants;

import java.util.OptionalDouble;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;


public class TurretSubsystem extends SubsystemBase{
    private TalonFX turretMotor = new TalonFX(TurretConstants.TurretMotorID);
    private DigitalInput turretZero = new DigitalInput(TurretConstants.TurretZeroID);
    private boolean isTurretZeroed;
    private PIDController rawVisionController = new PIDController(TurretConstants.rawVisionP, TurretConstants.rawVisionI, TurretConstants.rawVisionD);
    private double TurretTargetAngle = Double.NaN;
    private TurretControlMode turretMode;
    private double targetAngle;
    private double openLoopDemand;

    public TurretSubsystem()
    {
        rawVisionController.disableContinuousInput();
        turretMode = TurretControlMode.HOMING;
        isTurretZeroed =  false; 
        TalonFXConfiguration turretConfig = new TalonFXConfiguration();
        turretConfig.slot0.kP = TurretConstants.TurretP;
        turretConfig.slot0.kI = TurretConstants.TurretI;
        turretConfig.slot0.kD = TurretConstants.TurretD;
        turretConfig.slot0.kF = TurretConstants.TurretFF;
        turretConfig.motionAcceleration = TurretConstants.MotionMagicAcceleration;
        turretConfig.motionCruiseVelocity = TurretConstants.MotionMagicVelocity;
        turretConfig.motionCurveStrength= TurretConstants.MotionMagicCurve;
        turretConfig.forwardSoftLimitEnable = true; 
        turretConfig.reverseSoftLimitEnable = true; 
        turretMotor.enableVoltageCompensation(true);
        turretConfig.forwardSoftLimitThreshold = TurretConstants.TurretForwardSoftLimit* TurretConstants.TurretPostionCoffiecient;
        turretConfig.reverseSoftLimitThreshold = TurretConstants.TurretReverseSoftLimit* TurretConstants.TurretPostionCoffiecient;
        turretConfig.primaryPID.selectedFeedbackSensor = TalonFXFeedbackDevice.IntegratedSensor.toFeedbackDevice();
        turretMotor.configAllSettings(turretConfig);
        turretMotor.setNeutralMode(NeutralMode.Brake);

        

    }
    public void ZeroTurret(){
        if(!isTurretZeroed){
            if(turretZero.get()){
                turretMotor.setSelectedSensorPosition(0);
            }else if (!turretZero.get()){
                turretMotor.set(ControlMode.PercentOutput, 0.05);
            }
        }
    }

    public double getTurretAngle(){
        return turretMotor.getSelectedSensorPosition()*TurretConstants.TurretPostionCoffiecient;
    }
    public void setTurretAngle(double angle){
        turretMode = TurretControlMode.TARGETANGLE; 
        TurretTargetAngle = angle; 

    }
    public void UseVisionPID(double visionInput){
        turretMode = TurretControlMode.VISONPID;
         targetAngle = visionInput ;
   
    }
    public void turretOpenLoop(double turretOutput){
        turretMode = TurretControlMode.OPENLOOP; 
        openLoopDemand = turretOutput;
    }




    @Override
    public void periodic() {
        switch (turretMode){
            case DISABLED: 
            turretMotor.set(ControlMode.Disabled, 0);
            break; 
            case VISONPID: 
            double output = rawVisionController.calculate(targetAngle, 0);
            SmartDashboard.putNumber("Targeting Output", output);
            if(!rawVisionController.atSetpoint()) {
                if(output < 0) output = Math.min(-TurretConstants.turretMinRotation, output);
                else output = Math.max(TurretConstants.turretMinRotation, output);
            }
            if (getTurretAngle()>(TurretConstants.TurretForwardSoftLimit-30)& getTurretAngle()<TurretConstants.TurretReverseSoftLimit ){
                turretMotor.set(ControlMode.MotionMagic, (getTurretAngle()-360)/Constants.TurretConstants.TurretPostionCoffiecient);
            }else if (getTurretAngle()<(TurretConstants.TurretForwardSoftLimit)& getTurretAngle()>(TurretConstants.TurretReverseSoftLimit-30) ){
                turretMotor.set(ControlMode.MotionMagic, (getTurretAngle()+360)/Constants.TurretConstants.TurretPostionCoffiecient);}
                else  if(RobotContainer.blindlight.hasTarget()){
                    turretMotor.set(ControlMode.PercentOutput, output);
                }
           
            break; 
            case TARGETANGLE: 
            if (TurretTargetAngle < TurretConstants.TurretForwardSoftLimit&& TurretTargetAngle >TurretConstants.TurretReverseSoftLimit){
                turretMotor.set(ControlMode.MotionMagic, TurretTargetAngle/Constants.TurretConstants.TurretPostionCoffiecient);
            }else if (TurretTargetAngle < TurretConstants.TurretForwardSoftLimit&& TurretTargetAngle <TurretConstants.TurretReverseSoftLimit){
                turretMotor.set(ControlMode.MotionMagic, (TurretTargetAngle+360)/Constants.TurretConstants.TurretPostionCoffiecient);
            }else if (TurretTargetAngle > TurretConstants.TurretForwardSoftLimit&& TurretTargetAngle >TurretConstants.TurretReverseSoftLimit){
                turretMotor.set(ControlMode.MotionMagic, (TurretTargetAngle-360)/Constants.TurretConstants.TurretPostionCoffiecient);
            }
            
            break; 
            case OPENLOOP: 
            turretMotor.set(ControlMode.PercentOutput, openLoopDemand);
            break; 
            case HOMING: 
            if (!isTurretZeroed){
                ZeroTurret();
            }else turretMode = TurretControlMode.TARGETANGLE;
            break; 
            
        }


    }

    public enum TurretControlMode{
        DISABLED,
        VISONPID,
        TARGETANGLE,
        OPENLOOP, 
        HOMING
    }





    
    
    
}
