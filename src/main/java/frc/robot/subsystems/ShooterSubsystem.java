package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;

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
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.ExternalLib.JackInTheBotLib.math.MathUtils;
import frc.ExternalLib.JackInTheBotLib.robot.UpdateManager;
import com.revrobotics.AnalogInput;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;

import java.util.OptionalDouble;
import frc.ExternalLib.NorthwoodLib.NorthwoodDrivers.RevThroughBore;



public class ShooterSubsystem extends SubsystemBase implements UpdateManager.Updatable{
    private TalonFX Shooter = new TalonFX(Constants.ShooterConstants.ShooterID);
    private TalonFX ShooterFollower = new TalonFX(Constants.ShooterConstants.ShooterFollowerID);
    private CANSparkMax HoodMotor = new CANSparkMax(Constants.ShooterConstants.HoodID, MotorType.kBrushless);
    
    private RelativeEncoder HoodEncoder;
    private RevThroughBore HoodAbsEncoder = new RevThroughBore(Constants.ShooterConstants.HoodEncoderID, "HoodEncoder",Constants.ShooterConstants.HoodOffset );
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



    public ShooterSubsystem(){
        IsHoodHomed = false;

        Shooter.configFactoryDefault();
        ShooterFollower.configFactoryDefault();


        TalonFXConfiguration ShooterConfig = new TalonFXConfiguration();
        ShooterConfig.slot0.kP = Constants.ShooterConstants.ShooterP;
        ShooterConfig.slot0.kI = Constants.ShooterConstants.ShooterI;
        ShooterConfig.slot0.kD = Constants.ShooterConstants.ShooterD;
        ShooterConfig.primaryPID.selectedFeedbackSensor = TalonFXFeedbackDevice.IntegratedSensor.toFeedbackDevice();
        ShooterConfig.supplyCurrLimit.currentLimit = Constants.ShooterConstants.ShooterCurrentLimit;
        ShooterConfig.supplyCurrLimit.enable = true;
        ShooterConfig.voltageCompSaturation = 11.5;

        Shooter.configAllSettings(ShooterConfig);
        ShooterFollower.configAllSettings(ShooterConfig);

        ShooterFollower.follow(Shooter);
        



        


        HoodController = HoodMotor.getPIDController();
        HoodController.setFeedbackDevice(HoodEncoder);
        HoodController.setP(Constants.ShooterConstants.HoodP);
        HoodController.setI(Constants.ShooterConstants.HoodI);
        HoodController.setD(Constants.ShooterConstants.HoodD);
        HoodController.setIZone(Constants.ShooterConstants.HoodIZone);
        HoodController.setFF(Constants.ShooterConstants.HoodFF);
        HoodController.setOutputRange(Constants.ShooterConstants.HoodMinOutput, Constants.ShooterConstants.HoodMaxOutput);
        //HoodEncoder.setPositionConversionFactor(factor);


        //HoodEncoder = HoodMotor.getAlternateEncoder(encoderType, countsPerRev)
        ShuffleboardTab tab = Shuffleboard.getTab("Shooter");
        HoodAngleEntry = tab.add("Hood Angle", 0.0) 
        .withPosition(0, 0)
        .withSize(1, 1)
        .getEntry();
        tab.addNumber("Hood Raw Encoder",()-> HoodEncoder.getPosition())
        .withPosition(0, 2)
        .withSize(1, 1);



        
    }
    public void setFlywheelCurrentLimitEnabled(boolean enabled) {
        SupplyCurrentLimitConfiguration config = new SupplyCurrentLimitConfiguration();
        config.currentLimit = Constants.ShooterConstants.ShooterCurrentLimit;
        config.enable = enabled;
        Shooter.configSupplyCurrentLimit(config, 0);
        ShooterFollower.configSupplyCurrentLimit(config, 0);
        
    }

    public void RunShooter(double speed){
        double feedForward = (Constants.ShooterConstants.ShooterFF*speed+Constants.ShooterConstants.StaticFriction)/RobotController.getBatteryVoltage();
        Shooter.set(ControlMode.Velocity, -speed/Constants.ShooterConstants.ShooterVelocitySensorCoffiecient, DemandType.ArbitraryFeedForward, -feedForward);
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
    public boolean isHoodHomed() {
        return IsHoodHomed;
    }    
    public boolean isHoodAtTargetAngle() {
        OptionalDouble targetAngle = getHoodTargetAngle();
        double currentAngle = getHoodMotorAngle();

        if (targetAngle.isEmpty()) {
            return false;
        }

        return MathUtils.epsilonEquals(targetAngle.getAsDouble(), currentAngle, Math.toRadians(1.0));
    }
    
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

    }

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
                    HoodMotor.set(0.0);
                } else { 
                    double targetAngle = getHoodTargetAngle().getAsDouble();
                    targetAngle = MathUtils.clamp(targetAngle, ShooterConstants.HoodMinAngle, ShooterConstants.HoodMaxAngle);
                   
                     
                    /*double targetAngle = getHoodTargetAngle().getAsDouble();
                    targetAngle = MathUtils.clamp(targetAngle, Constants.ShooterConstants.HoodMinAngle, Constants.ShooterConstants.HoodMaxAngle);
                    double Output;
                   Output = HoodController.calculate(HoodEncoder.getDistanceDegrees(), targetAngle);
                    HoodMotor.set(Output);*/
                    setReferenceAngle(Units.degreesToRadians(targetAngle));

                }
                break;
            case PERCENT_OUTPUT:
                this.HoodMotor.set(hoodPercentOutput);
                break;
        }

        HoodAngleEntry.setDouble(Math.toDegrees(getHoodMotorAngle()));
    }
    public double getHoodMotorAngle(){
        return HoodAbsEncoder.getDistanceDegrees();
    }
    
    public void disableHood() {
        hoodControlMode = HoodControlMode.DISABLED;
        hoodTargetPosition = Double.NaN;
    }
    public void setHoodHomed(boolean target) {
        this.IsHoodHomed = target;
    }


public enum HoodControlMode {
        DISABLED,
        POSITION,
        PERCENT_OUTPUT
    }




    
}
