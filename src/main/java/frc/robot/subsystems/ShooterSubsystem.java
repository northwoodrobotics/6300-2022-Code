package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.music.Orchestra;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAlternateEncoder;
import com.revrobotics.SparkMaxAnalogSensor;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAnalogSensor;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
//import frc.ExternalLib.CitrusLib.Subsystem;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.ExternalLib.CitrusLib.ServoMotorSubsystem.PeriodicIO;
import frc.ExternalLib.JackInTheBotLib.math.MathUtils;
import frc.ExternalLib.JackInTheBotLib.robot.UpdateManager;
import com.revrobotics.AnalogInput;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;

import java.util.OptionalDouble;

import frc.ExternalLib.NorthwoodLib.NorthwoodDrivers.LinearServo;
import frc.ExternalLib.NorthwoodLib.NorthwoodDrivers.RevThroughBore;



public class ShooterSubsystem extends SubsystemBase implements UpdateManager.Updatable{
    private TalonFX Shooter = new TalonFX(Constants.ShooterConstants.ShooterID);
    private TalonFX ShooterFollower = new TalonFX(Constants.ShooterConstants.ShooterFollowerID);
    private CANSparkMax HoodMotor = new CANSparkMax(Constants.ShooterConstants.HoodID, MotorType.kBrushless);
    private Orchestra ShooterOrchestra = new Orchestra();
    //private Servo shooteServe1 = new Servo(ShooterConstants.HoodServoID);
    //private Servo ShooterServo = new Servo(ShooterConstants.HoodServoID);
    //private Servo ShooterServo2 = new Servo(ShooterConstants.HoodServo2ID);
    private static PeriodicIO mPeriodicIO = new PeriodicIO();

    

    private static final SendableChooser<String> SongChooser = new SendableChooser<>();
    public ShuffleboardTab music = Shuffleboard.getTab("Music");


    public String rickroll = "rickroll";
    public String gasgasgas = "gasgasgas";
    public String pokerface = "pokerface";
    public String stayinalive = "stayinalive";
    

    
    private RelativeEncoder HoodEncoder;
    //private RevThroughBore HoodAbsEncoder = new RevThroughBore(Constants.ShooterConstants.HoodEncoderID, "HoodEncoder",Constants.ShooterConstants.HoodOffset );
   // private SparkMaxAlternateEncoder HoodEncoder;
    private SparkMaxPIDController HoodController;
    private final NetworkTableEntry HoodAngleEntry;
    private boolean IsHoodHomed;
    private HoodControlMode hoodControlMode = HoodControlMode.DISABLED;
    private double hoodTargetPosition = Double.NaN;
    private double hoodPercentOutput = 0.0;
    private static final int ENCODER_RESET_ITERATIONS = 500;
    private static final double ENCODER_RESET_MAX_ANGULAR_VELOCITY = Math.toRadians(0.5);
    private double resetIteration = 0;
    private double referenceAngleRadians = 0;
    private double HoodZero = 0.0;
    private double HoodHalf = 0.5;
    private double HoodMax = 1.0;
    private double HoodMin = -1.0;




    public ShooterSubsystem(){
        IsHoodHomed = false;

        Shooter.configFactoryDefault();
        ShooterFollower.configFactoryDefault();
        //ShooterServo.setBounds(2.0,1.8,1.5, 1.2, 1.0);
        //ShooterServo2.setBounds(2.0,1.8,1.5, 1.2, 1.0);
      



        TalonFXConfiguration ShooterConfig = new TalonFXConfiguration();
        ShooterConfig.slot0.kP = Constants.ShooterConstants.ShooterP;
        ShooterConfig.slot0.kI = Constants.ShooterConstants.ShooterI;
        ShooterConfig.slot0.kD = Constants.ShooterConstants.ShooterD;
        ShooterConfig.primaryPID.selectedFeedbackSensor = TalonFXFeedbackDevice.IntegratedSensor.toFeedbackDevice();
        ShooterConfig.supplyCurrLimit.currentLimit = Constants.ShooterConstants.ShooterCurrentLimit;
        ShooterConfig.supplyCurrLimit.enable = true;
        ShooterConfig.voltageCompSaturation = 11.5;
        ShooterConfig.slot0.kF = ShooterConstants.ShooterFF;

        Shooter.configAllSettings(ShooterConfig);
        ShooterFollower.configAllSettings(ShooterConfig);

        ShooterFollower.follow(Shooter);
        ShooterFollower.setInverted(InvertType.InvertMotorOutput);
        SongChooser.setDefaultOption("rick roll",rickroll);
        SongChooser.addOption("Gas Gas GAS!", gasgasgas);
        SongChooser.addOption("PokerFace", pokerface);
        SongChooser.addOption("Stayin Alive", stayinalive);
        music.add(SongChooser);

        HoodEncoder = HoodMotor.getEncoder();
       
        



        


        HoodController = HoodMotor.getPIDController();
        HoodController.setFeedbackDevice(HoodEncoder);
        HoodController.setP(Constants.ShooterConstants.HoodP);
        HoodController.setI(Constants.ShooterConstants.HoodI);
        HoodController.setD(Constants.ShooterConstants.HoodD);
        HoodController.setIZone(Constants.ShooterConstants.HoodIZone);
        HoodController.setFF(Constants.ShooterConstants.HoodFF);
        HoodController.setOutputRange(Constants.ShooterConstants.HoodMinOutput, Constants.ShooterConstants.HoodMaxOutput);
        //HoodEncoder.setPositionConversionFactor(factor);*/

        ShooterOrchestra.addInstrument(Shooter);
        ShooterOrchestra.addInstrument(ShooterFollower);




        //HoodEncoder = HoodMotor.getAlternateEncoder(encoderType, countsPerRev)
        ShuffleboardTab tab = Shuffleboard.getTab("Shooter");
        HoodAngleEntry = tab.add("Hood Angle", 0.0) 
        .withPosition(0, 0)
        .withSize(1, 1)
        .getEntry();
        tab.addNumber("Hood Raw Encoder",()-> HoodEncoder.getPosition())
        .withPosition(0, 2)
        .withSize(1, 1);
        tab.addString("HoodControlMode", ()-> getControlMode())
        .withPosition(0, 3)
        .withSize(2, 2);
        tab.addNumber("HoodControlAngle", ()-> getHoodTargetAngle().orElse(Double.NaN))
        .withPosition(1, 2)
        .withSize(1, 1);

        


        
    }
    public void setFlywheelCurrentLimitEnabled(boolean enabled) {
        SupplyCurrentLimitConfiguration config = new SupplyCurrentLimitConfiguration();
        config.currentLimit = Constants.ShooterConstants.ShooterCurrentLimit;
        config.enable = enabled;
        Shooter.configSupplyCurrentLimit(config, 0);
        ShooterFollower.configSupplyCurrentLimit(config, 0);
        
    }
   /* public void percentoutput(double speed){
        Shooter.set(ControlMode.PercentOutput, speed);
    }*/


    public void setHoodMotorPower(double percent) {
        hoodControlMode = HoodControlMode.PERCENT_OUTPUT;
        hoodPercentOutput = percent;
    }


    




    

    public void RunShooter(double speed){
        //double feedForward = (Constants.ShooterConstants.ShooterFF*speed+Constants.ShooterConstants.StaticFriction)/RobotController.getBatteryVoltage();
        Shooter.set(ControlMode.Velocity, -speed/Constants.ShooterConstants.ShooterVelocitySensorCoffiecient);
    }
    public void PauseMusic(){
        ShooterOrchestra.pause();
    }
    public void LoadMusic(){
        ShooterOrchestra.loadMusic(SongChooser.getSelected());
    }
    public void PlayMusic(){
        ShooterOrchestra.play();
    }
    public void StopMusic(){
        ShooterOrchestra.stop();
        
    }
    public void stopFlywheel() {
        Shooter.set(ControlMode.Disabled, 0);
    }
    @Override
    public void update(double time, double dt) {
    }
  
    public OptionalDouble getHoodTargetAngle() {
        if (Double.isFinite(hoodTargetPosition)) {
            return OptionalDouble.of(hoodTargetPosition);
        } else {
            return OptionalDouble.empty();
        }
    }
    public void setHoodTargetAngle(double angle){
        hoodControlMode = HoodControlMode.POSITION;
        hoodTargetPosition = angle;
    }

    public double shooterSpeed(){
        return Shooter.getSelectedSensorVelocity()*Constants.ShooterConstants.ShooterVelocitySensorCoffiecient;
    }   


    public void MoveHood(double setpoint){
        HoodController.setReference(setpoint, ControlType.kPosition);
    }

    
    public boolean isHoodAtTargetAngle() {
        OptionalDouble targetAngle = getHoodTargetAngle();
        double currentAngle = getHoodTargetAngle().getAsDouble();

        if (targetAngle.isEmpty()) {
            return false;
        }

        return MathUtils.epsilonEquals(targetAngle.getAsDouble(), currentAngle, Math.toRadians(1.0));
    }
    /*
    @SuppressWarnings("SelfAssignment")
    public void setReferenceAngle(double refereneceAngleRadians){
        double currentAngleRadians = HoodEncoder.getPosition();
        if (HoodEncoder.getVelocity() < ENCODER_RESET_MAX_ANGULAR_VELOCITY) {
            if (++resetIteration >= ENCODER_RESET_ITERATIONS) {
                resetIteration = 0;
                double absoluteAngle = HoodAbsEncoder.getDistanceDegrees();
                HoodEncoder.setPosition(absoluteAngle);
                currentAngleRadians = absoluteAngle;
            }
        } else {
            resetIteration = 0;
        }
        
        double currentAngleRadiansMod = currentAngleRadians % (2.0 * Math.PI);
        if (currentAngleRadiansMod < 0.0) {
            currentAngleRadiansMod += 2.0 * Math.PI;
        }

        // The reference angle has the range [0, 2pi) but the Neo's encoder can go above that
        double adjustedReferenceAngleRadians = referenceAngleRadians + currentAngleRadians - currentAngleRadiansMod;
        if (referenceAngleRadians - currentAngleRadiansMod > Math.PI) {
            adjustedReferenceAngleRadians -= 2.0 * Math.PI;
        } else if (referenceAngleRadians - currentAngleRadiansMod < -Math.PI) {
            adjustedReferenceAngleRadians += 2.0 * Math.PI;
        }

        this.referenceAngleRadians = referenceAngleRadians;

        HoodController.setReference(adjustedReferenceAngleRadians, ControlType.kPosition);

    }*/

    
    

    @Override
    public void periodic() {
        switch (hoodControlMode) {
            case DISABLED:
                HoodMotor.set(0.0);
                break;
            case POSITION:
                if (!IsHoodHomed) {
                    break;
                }

                if (getHoodTargetAngle().isEmpty()) {
                    HoodController.setReference(0, ControlType.kPosition);
                } else { 
                    double targetAngle = getHoodTargetAngle().getAsDouble();
                    targetAngle = MathUtils.clamp(targetAngle, ShooterConstants.HoodMinAngle, ShooterConstants.HoodMaxAngle);

                    HoodController.setReference(targetAngle, ControlType.kPosition);
                   
                     
                  //  double targetAngle = getHoodTargetAngle().getAsDouble();
                   // targetAngle = MathUtils.clamp(targetAngle, Constants.ShooterConstants.HoodMinAngle, Constants.ShooterConstants.HoodMaxAngle);
                    
                   //Output = HoodController.calculate(HoodEncoder.getPosition(), targetAngle);
                    //HoodMotor.set(Output);*//*
                    //setReferenceAngle(Units.degreesToRadians(targetAngle));

                }
                break;
            case PERCENT_OUTPUT:
                this.HoodMotor.set(hoodPercentOutput);
                break;
        }

        //HoodAngleEntry.setDouble(getHoodTargetAngle().getAsDouble());

        
    }
    
    public double getHoodMotorAngle(){
        return HoodEncoder.getPosition();
    }
    
    public void disableHood() {
        hoodControlMode = HoodControlMode.DISABLED;
        hoodTargetPosition = Double.NaN;
    }
    public void setHoodHomed(boolean target) {
        this.IsHoodHomed = target;
    }
    public boolean isHoodHomed() {
        return IsHoodHomed;
    } 
    public void zeroHoodMotor() {
        this.IsHoodHomed = true;

        double sensorPosition = (0);
        HoodEncoder.setPosition((int) sensorPosition);
    }
    public String getControlMode(){
        return hoodControlMode.toString();
    }
    


public enum HoodControlMode {
        DISABLED,
        POSITION,
        PERCENT_OUTPUT
    }

    public enum ShooterControlMode{
        MUSIC, SHOOT
    }




    
}
