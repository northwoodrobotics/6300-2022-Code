package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.google.errorprone.annotations.concurrent.GuardedBy;
import com.revrobotics.CANSparkMax;
import com.revrobotics.EncoderType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAlternateEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ClimberConstants;

public class ClimberSubsystem extends SubsystemBase{
    //private TalonSRX TestClimbMotor = new TalonSRX(31);
    //real stuff
    private CANSparkMax ClimbMotor1 = new CANSparkMax(Constants.ClimberConstants.ClimbMotor1, MotorType.kBrushless);
    private CANSparkMax ClimbMotor2 = new CANSparkMax(Constants.ClimberConstants.ClimbMotor2, MotorType.kBrushless);
    private Solenoid lockSolenoid = new Solenoid(PneumaticsModuleType.REVPH, ClimberConstants.ClimbSolenoid);
    private Solenoid BalanceSolenoid = new Solenoid(PneumaticsModuleType.REVPH, ClimberConstants.BalanceSolenoid);
    //private Solenoid CatchSolenoid = new Solenoid(PneumaticsModuleType.REVPH, ClimberConstants.CatchSolenoid);
    private SparkMaxPIDController Climb1Controller;
    private SparkMaxPIDController Climb2Controller;
    private RelativeEncoder Climb1Encoder;
    private RelativeEncoder Climb2Encoder;
    private final Object stateLock = new Object();

    //@GuardedBy("stateLock")
    //private boolean LatchOn = false;
    @GuardedBy("stateLock")
    private boolean ClimbUp = false;
    

    public ClimberSubsystem(){
        setBreakModes();
        Climb1Controller = ClimbMotor1.getPIDController();
        Climb2Controller = ClimbMotor2.getPIDController();
        Climb1Encoder = ClimbMotor1.getAlternateEncoder(ClimberConstants.RevEncoder_CountsPer_Rev);
        Climb2Encoder = ClimbMotor2.getAlternateEncoder(ClimberConstants.RevEncoder_CountsPer_Rev);


        
    }
    
    public void setBreakModes(){
        ClimbMotor1.setIdleMode(IdleMode.kBrake);
        ClimbMotor2.setIdleMode(IdleMode.kBrake);
    }

   



    
    
}
