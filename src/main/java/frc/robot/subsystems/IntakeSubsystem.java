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


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.google.errorprone.annotations.concurrent.GuardedBy;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.MjpegServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoMode.PixelFormat;





public class IntakeSubsystem extends SubsystemBase{
    // thie intake consisted of a single double solenoid, and 775 pro run on a talon SRX 
    private TalonSRX intakeMotor = new TalonSRX(Constants.IntakeConstants.IntakeMotorID);
    private DoubleSolenoid intakeSolenoid = new DoubleSolenoid(31, PneumaticsModuleType.REVPH, Constants.IntakeConstants.IntakeSolenoidID2, IntakeConstants.IntakeSolenoidID);

    /*private UsbCamera intakeCam = new UsbCamera("intakeCAM", 0);
    private MjpegServer camServer = new MjpegServer("intakeCam Server", 1181);
    private CvSink cvSink = new CvSink("USBCam OpenCV");
    CvSource outputStream = new CvSource("Blur", PixelFormat.kMJPEG, 640, 480, 30);*/
    //private final Object stateLock = new Object();

 
    private double motorOutput = 0.0;
    
 
    //private Value intakeExtended;
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
        //intakeExtended = Value.kReverse;
       // cvSink.setSource(intakeCam);
       CameraServer.startAutomaticCapture();
       intakeMotor.setStatusFramePeriod(1, 150); // slow down the motor updates, the inake has no control loop, so it doesn't need to update every 10ms
        

        intakeMotor.setNeutralMode(NeutralMode.Brake); // brake mode because it feels nice
        intakeMotor.configPeakCurrentLimit(25); // current limit, as 775pros burn out like no tomorrow 
        intakeMotor.configPeakCurrentDuration(200);
        intakeMotor.configContinuousCurrentLimit(15);
        intakeMotor.enableCurrentLimit(true);
        
    }


    public Value isIntakeExtended(){
        {
           return intakeSolenoid.get();
        }


    }

    public void setIntakeExtension(Value IntakeState){
            intakeSolenoid.set(IntakeState);
            //this.intakeExtended = IntakeState;
        
        
    }
    public void setMotorOutput(double Output){
        //synchronized(stateLock){
            this.motorOutput = Output;
            intakeMotor.set(ControlMode.PercentOutput, Output);
        //}
    }
    public double getMotorOutput(){
        //synchronized(stateLock){
            return motorOutput;
        //}
    }

    @Override 
    public void periodic(){
        //intakeSolenoid.set(this.intakeExtended);
        //intakeMotor.set(motorOutput);
        IntakeMotorSpeed.setDouble(getMotorOutput());
        //IntakeExtendedEntry.setValue(isIntakeExtended());
    }



    





    
    
}
