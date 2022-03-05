package frc.robot.subsystems;




import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;
import frc.ExternalLib.JackInTheBotLib.robot.UpdateManager;

import com.google.errorprone.annotations.concurrent.GuardedBy;




public class IntakeSubsystem extends SubsystemBase{
    private CANSparkMax intakeMotor = new CANSparkMax(Constants.IntakeConstants.IntakeMotorID, MotorType.kBrushless);
    private DoubleSolenoid intakeSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, Constants.IntakeConstants.IntakeSolenoidID, IntakeConstants.IntakeSolenoidID2);


    private final Object stateLock = new Object();

 
    private double motorOutput = 0.0;
    
 
    private Value intakeExtended = Value.kReverse; 

    private final NetworkTableEntry IntakeMotorSpeed;
    private final NetworkTableEntry IntakeExtendedEntry;

    public IntakeSubsystem(){
        ShuffleboardTab tab = Shuffleboard.getTab("Intake");
        IntakeMotorSpeed =  tab.add("motorspeed", 0.0)
        .withPosition(0, 0)
        .withSize(1, 1)
        .getEntry();
       IntakeExtendedEntry = tab.add("Is Intake Extended", false)
        .withPosition(0, 1)
        .withSize(1, 1)
        .getEntry();

        intakeMotor.setIdleMode(IdleMode.kBrake);
        
    }


    public Value isIntakeExtended(){
        {
           return intakeSolenoid.get();
        }


    }

    public void setIntakeExtension(Value IntakeState){
      
            this.intakeExtended = IntakeState;
        
        
    }
    public void setMotorOutput(double Output){
        synchronized(stateLock){
            this.motorOutput = Output;
            intakeMotor.set(Output);
        }
    }
    public double getMotorOutput(){
        synchronized(stateLock){
            return motorOutput;
        }
    }

    @Override 
    public void periodic(){
        //intakeMotor.set(motorOutput);
        IntakeMotorSpeed.setDouble(getMotorOutput());
        //IntakeExtendedEntry.setValue(isIntakeExtended());
    }



    





    
    
}
