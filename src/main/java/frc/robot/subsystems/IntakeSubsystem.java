package frc.robot.subsystems;




import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.ExternalLib.JackInTheBotLib.robot.UpdateManager;

import com.google.errorprone.annotations.concurrent.GuardedBy;




public class IntakeSubsystem extends SubsystemBase implements UpdateManager.Updatable{
    private CANSparkMax intakeMotor = new CANSparkMax(Constants.IntakeConstants.IntakeMotorID, MotorType.kBrushless);
    private Solenoid intakeSolenoid = new Solenoid(PneumaticsModuleType.REVPH, Constants.IntakeConstants.IntakeSolenoidID);


    private final Object stateLock = new Object();

    @GuardedBy ("stateLock")
    private double motorOutput = 0.0;
    
    @GuardedBy("stateLock")
    private boolean intakeExtended = false; 

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
    }
    @Override
    public void update(double time, double dt){
        double localIntakeOutput;
        boolean localIntakeExtended;
        synchronized(stateLock){

            if(localIntakeExtended = true){
                localIntakeOutput = 1.0;
            }
            else localIntakeOutput = 0;
            localIntakeOutput = motorOutput;
            localIntakeExtended = intakeExtended;


        }


        intakeMotor.set(localIntakeOutput);
        if (localIntakeExtended != intakeSolenoid.get()) {
            intakeSolenoid.set(localIntakeExtended);
        }


    }

    public boolean isIntakeExtended(){
        synchronized(stateLock){
            return intakeExtended;
        }


    }

    public void setIntakeExtension(boolean IntakeState){
        synchronized(stateLock){
            this.intakeExtended = IntakeState;
        }
        
    }
    public void setMotorOutput(double Output){
        synchronized(stateLock){
            this.motorOutput = Output;
        }
    }
    public double getMotorOutput(){
        synchronized(stateLock){
            return motorOutput;
        }
    }

    @Override 
    public void periodic(){
        IntakeMotorSpeed.setDouble(getMotorOutput());
        IntakeExtendedEntry.setBoolean(isIntakeExtended());
    }



    





    
    
}
