package frc.robot.subsystems;




import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;

import java.util.OptionalDouble;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;





public class IntakeSubsystem extends SubsystemBase{
    
    private TalonFX intakeMotor = new TalonFX(Constants.IntakeConstants.IntakeMotorID);
    private TalonFX WristMotor = new TalonFX(IntakeConstants.WristMotorID);
    private DigitalInput WristLimit = new DigitalInput(IntakeConstants.WristLimitSwitch);
    
    private double motorOutput = 0.0;
    private double wristPercentOutput = 0.25;
    private boolean isWristHome;
    private double wristTargetPosition = Double.NaN;
    private IntakeModes intakeMode; 

    
 

    private final NetworkTableEntry IntakeMotorSpeed;
 

    public IntakeSubsystem(){
        isWristHome = false;
        intakeMode = IntakeModes.IntakeDisabled;

        ShuffleboardTab tab = Shuffleboard.getTab("Intake");
        IntakeMotorSpeed =  tab.add("motorspeed", 0.0)
        .withPosition(0, 0)
        .withSize(1, 1)
        .getEntry();


        
        intakeMotor.setStatusFramePeriod(1, 150); // slow down the motor updates, the inake has no control loop, so it doesn't need to update every 10ms
        intakeMotor.setNeutralMode(NeutralMode.Brake); // brake mode because it feels nice
        ShutTalonUP(intakeMotor);
        WristMotor.setInverted(true);
        WristMotor.setNeutralMode(NeutralMode.Brake);
        TalonFXConfiguration WristConfig = new TalonFXConfiguration();
        WristConfig.slot0.kP = IntakeConstants.WristP;
        WristConfig.slot0.kI = IntakeConstants.WristI;
        WristConfig.slot0.kD = IntakeConstants.WristD;
        WristConfig.slot0.kF = IntakeConstants.WristFF;
        WristConfig.motionAcceleration = IntakeConstants.MotionMagicAcceleration; 
        WristConfig.motionCruiseVelocity = IntakeConstants.MotionMagicVelocity;
        WristConfig.motionCurveStrength = IntakeConstants.MotionMagicCurve;
        WristConfig.primaryPID.selectedFeedbackSensor = TalonFXFeedbackDevice.IntegratedSensor.toFeedbackDevice();
        WristConfig.supplyCurrLimit.currentLimit = 15;  // current limited, as its never going to need more than this
        WristConfig.supplyCurrLimit.enable = true;

        WristMotor.configAllSettings(WristConfig);



        
        
    }
    public void ShutTalonUP(TalonFX targetTalon){
        targetTalon.setStatusFramePeriod(4, 251);
        targetTalon.setStatusFramePeriod(10, 251);
        targetTalon.setStatusFramePeriod(12, 251);
        targetTalon.setStatusFramePeriod(13, 251);
        targetTalon.setStatusFramePeriod(21, 251);
        targetTalon.setStatusFramePeriod(4, 251);

    }
    public void ZeroWrist(){
        if (isWristHome){
            
        }else if (WristLimit.get()){
            WristMotor.setSelectedSensorPosition(0);
            isWristHome = true;
        }else if (!WristLimit.get()){
            WristMotor.set(ControlMode.PercentOutput, -wristPercentOutput);
        }
    }

    public OptionalDouble getWristTargetPosition() {
        if (Double.isFinite(wristTargetPosition)) {
            return OptionalDouble.of(wristTargetPosition);
        } else {
            return OptionalDouble.empty();
        }
    }
    public void IntakeDown(){
        intakeMode = IntakeModes.IntakeDeployed;
    }
    public void IntakeUp(){
        intakeMode = IntakeModes.IntakeUp;
    }
    public void IntakeDribble(){
        intakeMode = IntakeModes.IntakeDribble;
    }


    public void WristUp(){
        WristMotor.set(TalonFXControlMode.MotionMagic, 0*IntakeConstants.WristPositionSensorCoffiecient);
    }
    public void WristDribble(){
        WristMotor.set(TalonFXControlMode.MotionMagic, 35*IntakeConstants.WristPositionSensorCoffiecient);
    }
    public void WristDeployed(){
        WristMotor.set(TalonFXControlMode.MotionMagic, 90*IntakeConstants.WristPositionSensorCoffiecient);
    }
    



    public void setMotorOutput(double Output){
        
            this.motorOutput = Output;
            intakeMotor.set(ControlMode.PercentOutput, Output);
       
    }
    public double getMotorOutput(){
       
            return motorOutput;
        
    }

    @Override 
    public void periodic(){
        IntakeMotorSpeed.setDouble(getMotorOutput());

        switch(intakeMode){
            case IntakeDisabled:
            intakeMotor.set(ControlMode.Disabled, 0);
            WristUp();
            break;
            case IntakeDribble: 
            intakeMotor.set(ControlMode.PercentOutput, 1);
            WristDribble();
            break;
            case IntakeDeployed:
            intakeMotor.set(ControlMode.PercentOutput, 1);
            WristDeployed();
            break; 
            case IntakeUp: 
            intakeMotor.set(ControlMode.PercentOutput, 1);
            WristUp();
            
        }


       
    }

    private double talonUnitsToHoodAngle(double talonUnits) {
        return -talonUnits / 2048 * (2 * Math.PI);
    } // converts falcon 500 encoder reading to hood angle

    private double angleToTalonUnits(double angle) {
        return angle * 2048 / (2 * Math.PI) ;
    }// reverse of above function 
    
    private enum IntakeModes{
        IntakeUp, IntakeDeployed, IntakeDribble, IntakeDisabled
    }



    





    
    
}
