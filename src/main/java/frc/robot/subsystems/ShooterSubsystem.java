package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.ExternalLib.JackInTheBotLib.math.MathUtils;


import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;

import java.util.OptionalDouble;
import java.util.Map;




public class ShooterSubsystem extends SubsystemBase {
    private TalonFX Shooter = new TalonFX(Constants.ShooterConstants.ShooterID);
    private TalonFX ShooterFollower = new TalonFX(Constants.ShooterConstants.ShooterFollowerID);
    private TalonFX HoodMotor = new TalonFX(Constants.ShooterConstants.HoodID);
  

   
    

    private static final SendableChooser<String> SongChooser = new SendableChooser<>();
    public ShuffleboardTab music = Shuffleboard.getTab("Music");


    public String rickroll = "rickroll";
    public String gasgasgas = "gasgasgas";
    public String pokerface = "pokerface";
    public String stayinalive = "stayinalive";
    

    
    private final NetworkTableEntry HoodAngleEntry;
    private boolean IsHoodHomed;
    private HoodControlMode hoodControlMode = HoodControlMode.DISABLED;
    private double hoodTargetPosition = Double.NaN;
    private double hoodPercentOutput = 0.0;

    private double flyWheelOffset = 0.0; 
    private double hoodOffset = 0.0; 

    private final NetworkTableEntry flyWheelOffsetEntry;
    private final NetworkTableEntry hoodOffsetEntry;
 




    public ShooterSubsystem(){
        IsHoodHomed = true;

        Shooter.configFactoryDefault();
        ShooterFollower.configFactoryDefault();
    
        Shooter.setStatusFramePeriod(21, 200);
        ShooterFollower.setStatusFramePeriod(21, 200);

        
        /* here we set up the flywheel control loop. the flywheel is run using PIDF closed loop control, this just means it follows a sensor.
        If you are creating another flywheel shooter with Falcon500s, this code should work with minimal changes. 
        Here we are using Velocity PIDF with the Falcon500s integrated sensor.
        The Falcon 500 has a lot of horsepower behind it, and gets the flywheel up to speed rather quickly. if the 2022 bot is still around, 
        Fire it up and hear the flywheel scream. 

         */
        TalonFXConfiguration ShooterConfig = new TalonFXConfiguration();
        ShooterConfig.slot0.kP = Constants.ShooterConstants.ShooterP; // give the Falcon its P constant.
        ShooterConfig.slot0.kI = Constants.ShooterConstants.ShooterI; // give the Falcon its I constant, this is at zero, just P was enough for this loop.
        ShooterConfig.slot0.kD = Constants.ShooterConstants.ShooterD; // give the Falcon its D constant. again this is just zero
        ShooterConfig.primaryPID.selectedFeedbackSensor = TalonFXFeedbackDevice.IntegratedSensor.toFeedbackDevice(); // telling the falcon to use its internal encoder
        ShooterConfig.supplyCurrLimit.currentLimit = Constants.ShooterConstants.ShooterCurrentLimit; // this is for when the flywheel is off, we current limit to 10 amps, to keep it from randomly sucking power. 
        ShooterConfig.supplyCurrLimit.enable = true; // turn on the current limt
        ShooterConfig.voltageCompSaturation = 11.5; // what this does is attempt to allways apply the same voltage to the flywheel, regardless of battery voltage, this works until your battery getst down to 10ish amps
        ShooterConfig.slot0.kF = ShooterConstants.ShooterFF; // F term for the PIDF loop
        Shooter.enableVoltageCompensation(true);
        ShooterFollower.enableVoltageCompensation(true);

        Shooter.configAllSettings(ShooterConfig); // apply the configuration for the shooter 
        ShooterFollower.configAllSettings(ShooterConfig); // apply the config to the follower motor (mostly for the current limit)

        ShooterFollower.follow(Shooter);// tells the follower motor to simply do whatever the master is doing 
        ShooterFollower.setInverted(InvertType.InvertMotorOutput); // inverts the follower, so that they spin in the same dirrection 
        SongChooser.setDefaultOption("rick roll",rickroll);// falcons can play music, this is just some fun stuff, not particulaly relevant 
        SongChooser.addOption("Gas Gas GAS!", gasgasgas);
        SongChooser.addOption("PokerFace", pokerface);
        SongChooser.addOption("Stayin Alive", stayinalive);
        music.add(SongChooser);

        

        /* the hood uses motion magic control. This is very similar to positon PIDF, but with some extra sauce added. 
        Positon PIDF is basicaly just, YEET to the setpoint, so 
        if the error is large, then 
        P gets it mostly there, so { current point -------------> setpoint } its hard to get it to respond fast without having it overshoot and overcorrect
        then D will curve it or slow it down (kind off)  {the D term is a constants, which is multiplied by the Derivative (slope) in change of position/error }, 
        so you try to get this to zero as P sends you to your target. 

        This means that your loop will end up in some unhappy medium between respose time and accuracy. if you want an super accurate loop, then you need slow ish P and small D terms to get a good loop. A fast responding loop may be unstable. 
        Motion magic fixes this problem. what it is a velocity loop inside of a position loop. for example, 
        if you are biking, you are going to start, then accelerate to a confortable speed, but when you get close to your destination, you are going to slow down. 
        this is much more ideal, as your loop will respond faster, and be smoother. less overshooting if tuned properly, and far more accurate than a snappy PID loop. 
        
        Motion magic on a Falcon 500: 
        tuning this is a bit of an art, and will be different based on your mechanism, but in code, the process is very similar to the shooter PIDF
        the only added parameters are the acceleration and velocity parameters, so how fast can the mechanism accelerate, and how fast can it go. this is kept in Talon units.
        because I am lazy. 

        */

        

        TalonFXConfiguration HoodConfig = new TalonFXConfiguration();
        HoodConfig.slot0.kP = ShooterConstants.HoodP;
        HoodConfig.slot0.kI = ShooterConstants.HoodI;
        HoodConfig.slot0.kD = ShooterConstants.HoodD;
        HoodConfig.slot0.kF = ShooterConstants.HoodFF;
        HoodConfig.motionAcceleration = ShooterConstants.MotionMagicAcceleration; 
        HoodConfig.motionCruiseVelocity = ShooterConstants.MotionMagicVelocity;
        HoodConfig.motionCurveStrength = ShooterConstants.MotionMagicCurve;
        HoodConfig.primaryPID.selectedFeedbackSensor = TalonFXFeedbackDevice.IntegratedSensor.toFeedbackDevice();
        HoodConfig.supplyCurrLimit.currentLimit = 15;  // current limited, as its never going to need more than this
        HoodConfig.supplyCurrLimit.enable = true;

        HoodMotor.configAllSettings(HoodConfig);
        HoodMotor.setNeutralMode(NeutralMode.Brake); // set it in brake mode, essentaly lightly stalling the motor to keep in place. 


       




        

        // shufflboard is used on the driverstation to see robot data. this page is for the shooter, mostly for tuning. 
        ShuffleboardTab tab = Shuffleboard.getTab("Shooter");
        HoodAngleEntry = tab.add("Hood Angle", 0.0) 
        .withPosition(0, 0)
        .withSize(1, 1)
        .getEntry();
        tab.addNumber("Hood Raw Encoder",()-> HoodMotor.getSelectedSensorPosition()) // gets the raw position of the hood in talon units
        .withPosition(0, 2)
        .withSize(1, 1);
        tab.addString("HoodControlMode", ()-> getControlMode()) // gets the control mode of the hood
        .withPosition(0, 3)
        .withSize(2, 2);
        tab.addNumber("HoodControlAngle", ()-> getHoodTargetAngle().orElse(Double.NaN)) // gets the target angle, or "setpoint" of the hood
        .withPosition(1, 2)
        .withSize(1, 1);
        tab.addBoolean("HoodIsHomed", ()->this.IsHoodHomed) // checks if it has been zeroed. in 2022, we zeroed at the start of the match manualy before turing it on, in other years, we may use limit switches
        .withPosition(1, 2)
        .withSize(1, 1);
        tab.addNumber("HoodConverted", ()-> getHoodAngle()) // converted true hood angle. 
        .withPosition(1, 3)
        .withSize(1, 1);
        flyWheelOffsetEntry = Shuffleboard.getTab("master")
        .add("SpeedOffset", 0.0).withWidget(BuiltInWidgets.kNumberSlider)
        .withProperties(Map.of("min", -1000.0, "max", 1000.0, "Block increment", 25.0)).withPosition(2, 1)
        .getEntry();
        hoodOffsetEntry = Shuffleboard.getTab("master")
        .add("AngleOffset", 0.0).withWidget(BuiltInWidgets.kNumberSlider)
        .withProperties(Map.of("min", -10, "max", 10, "Block increment", 25.0)).withPosition(2, 1)
        .getEntry();


        


        
    }
    // function to turn on or off current limit
    public void setFlywheelCurrentLimitEnabled(boolean enabled) { 
        SupplyCurrentLimitConfiguration config = new SupplyCurrentLimitConfiguration();
        config.currentLimit = Constants.ShooterConstants.ShooterCurrentLimit;
        config.enable = enabled;
        Shooter.configSupplyCurrentLimit(config, 0);
        ShooterFollower.configSupplyCurrentLimit(config, 0);
        
    }
    

    // hood falcon to percent output control, largely unused
    public void setHoodMotorPower(double percent) {
        hoodControlMode = HoodControlMode.PERCENT_OUTPUT;
        hoodPercentOutput = percent;
    }


    




    
    // runs shooter at set RPM
    public void RunShooter(double speed){
       
        Shooter.set(ControlMode.Velocity, -speed-flyWheelOffset/Constants.ShooterConstants.ShooterVelocitySensorCoffiecient);
    }
 
    public void stopFlywheel() {
        Shooter.set(ControlMode.Disabled, 0);
    }

    // checkes if the flywheel is within the target velocity 
    public boolean isFlyWheelAtTargetVelocity(){
        return MathUtils.epsilonEquals(shooterSpeed(), getShooterTargetVelocity(), 300);
    }
    // returns hood angle 
    public OptionalDouble getHoodTargetAngle() {
        if (Double.isFinite(hoodTargetPosition)) {
            return OptionalDouble.of(hoodTargetPosition);
        } else {
            return OptionalDouble.empty();
        }
    }
    // sets hood target position 
    public void setHoodTargetAngle(double angle){
        hoodControlMode = HoodControlMode.POSITION;
        hoodTargetPosition = angle;
    }
    // returns true hood RPM, used for tuning 
    public double shooterSpeed(){
        return Shooter.getSelectedSensorVelocity()*Constants.ShooterConstants.ShooterVelocitySensorCoffiecient;
    }   

    // legacy, not used anymore. 
    public void MoveHood(double setpoint){
        //HoodMotor.getPIDController().setReference(setpoint, ControlType.kPosition);
        HoodMotor.set(ControlMode.MotionMagic, setpoint* ShooterConstants.HoodPositionSensorCoffiecient);
    }
    // gets how fast the hood is moving, also used for troubleshooting. 
    public double getHoodVelocity(){
        return HoodMotor.getSelectedSensorVelocity()*ShooterConstants.HoodVelocitySensorCoffiecient;

    }

    // checks if hood is at target angle, for shooter logic, but unused as of now
   public boolean isHoodAtTargetAngle() {
        OptionalDouble targetAngle = getHoodTargetAngle();
        double currentAngle = getHoodTargetAngle().getAsDouble();

        if (targetAngle.isEmpty()) {
            return false;
        }

        return MathUtils.epsilonEquals(targetAngle.getAsDouble(), currentAngle, Math.toRadians(0.1));
    }
    public double getShooterTargetVelocity(){
        return Shooter.getClosedLoopTarget()*Constants.ShooterConstants.ShooterVelocitySensorCoffiecient;
    }
  

    
    
    // this function is called as part of the subssytem class. basically this is allways running, and therefor so is the hood. it checks its mode, and where it should be every 100ms, 
    // this is done so that at no point the hood will not respond to commands. 
    // this logic can be reused and renamed (aside from the period function, that kind of has to stay to work) for any sort of arm system. 
    @Override
    public void periodic() {
        switch (hoodControlMode) {
            case DISABLED:
            HoodMotor.set(ControlMode.Disabled, 0.0);
                break;
            case POSITION: // if the hood is not zeroed, it will not run
                if (!IsHoodHomed) {
                    break;
                }
                // if there is no target angle, the hood will stay at the zero
                if (getHoodTargetAngle().isEmpty()) {
                    //HoodController.setReference(0, ControlType.kPosition);
                    HoodMotor.set(ControlMode.Disabled, 0.0);
                } else { 
                    
                    double targetAngle = getHoodTargetAngle().getAsDouble() + hoodOffset; // recives target angle 
                    targetAngle = MathUtils.clamp(targetAngle, ShooterConstants.HoodMinAngle, ShooterConstants.HoodMaxAngle);// this function was borrowed from 2910 Jack-In-The-Bot, and simplly forces the hood to stay within its own min an max angles
                
                    HoodMotor.set(ControlMode.MotionMagic, angleToTalonUnits(targetAngle)); // Hood Falcon recives the target angle 
                 

                }
                break;
            case PERCENT_OUTPUT:
                this.HoodMotor.set(ControlMode.PercentOutput,hoodPercentOutput);; // runs hood at certain percent output
                break;
        }

       
        flyWheelOffset = flyWheelOffsetEntry.getDouble(0.0);
        hoodOffset = hoodOffsetEntry.getDouble(0.0);
    }
   


    private double talonUnitsToHoodAngle(double talonUnits) {
        return -talonUnits / 2048 * (2 * Math.PI);
    } // converts falcon 500 encoder reading to hood angle

    private double angleToTalonUnits(double angle) {
        return angle * 2048 / (2 * Math.PI) ;
    }// reverse of above function 
    

    public void disableHood() {
        hoodControlMode = HoodControlMode.DISABLED;
        hoodTargetPosition = Double.NaN;
    }// turns hood off, and zeros the target position
    public void setHoodHomed(boolean target) {
        this.IsHoodHomed = target;
    } 
    public boolean isHoodHomed() {
        return IsHoodHomed;
    } 
    public String getControlMode(){
        return hoodControlMode.toString();
    }
    public double getHoodAngle(){
        return talonUnitsToHoodAngle(HoodMotor.getSelectedSensorPosition());
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
