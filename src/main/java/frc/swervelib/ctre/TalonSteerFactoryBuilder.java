package frc.swervelib.ctre;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardContainer;
import frc.swervelib.AbsoluteEncoderFactory;
import frc.swervelib.EnclosedSteerController;
import frc.swervelib.EnclosedSteerControllerFactory;
import frc.swervelib.ModuleConfiguration;

public class TalonSteerFactoryBuilder {

    private static final int CAN_TIMEOUT_MS = 250;
    private static final int STATUS_FRAME_GENERAL_PERIOD_MS = 250;
    
    private static final double TICKS_PER_ROTATION = 4028.0;
    private static final double SRX_MAG_TICS = 4028.0;
    private static final double VERSAPLANETARY_TICS = 2048.0;

    private double proportionalConstant = Double.NaN;
    private double integralConstant = Double.NaN;
    private double derivativeConstant = Double.NaN;

    // Motion magic configuration
    private double velocityConstant = Double.NaN;
    private double accelerationConstant = Double.NaN;
    private double staticConstant = Double.NaN;

    private double nominalVoltage = Double.NaN;
    private double currentLimit = Double.NaN;

    public TalonSteerFactoryBuilder withPidConstants(double proportional, double integral, double derivative) {
        this.proportionalConstant = proportional;
        this.integralConstant = integral;
        this.derivativeConstant = derivative;
        return this;
    }
    public boolean hasPidConstants() {
        return Double.isFinite(proportionalConstant) && Double.isFinite(integralConstant) && Double.isFinite(derivativeConstant);
    }

    public TalonSteerFactoryBuilder withMotionMagic(double velocityConstant, double accelerationConstant, double staticConstant) {
        this.velocityConstant = velocityConstant;
        this.accelerationConstant = accelerationConstant;
        this.staticConstant = staticConstant;
        return this;
    }
    public boolean hasMotionMagic() {
        return Double.isFinite(velocityConstant) && Double.isFinite(accelerationConstant) && Double.isFinite(staticConstant);
    }

    
    public TalonSteerFactoryBuilder withVoltageCompensation(double nominalVoltage) {
        this.nominalVoltage = nominalVoltage;
        return this;
    }

    public boolean hasVoltageCompensation() {
        return Double.isFinite(nominalVoltage);
    }

    public TalonSteerFactoryBuilder withCurrentLimit(double currentLimit) {
        this.currentLimit = currentLimit;
        return this;
    }
    public boolean hasCurrentLimit() {
        return Double.isFinite(currentLimit);
    }
    public <T> EnclosedSteerControllerFactory<ControllerImplementation, TalonSteerConfiguration<T>>build(AbsoluteEncoderFactory<T> absoluteEncoderFactory){
        return new FactoryImplementation<>(absoluteEncoderFactory);
    } 

    private class FactoryImplementation<T> implements EnclosedSteerControllerFactory<ControllerImplementation, TalonSteerConfiguration<T>> {
        private final AbsoluteEncoderFactory<T> encoderFactory;

        private FactoryImplementation(AbsoluteEncoderFactory<T> encoderFactory) {
            this.encoderFactory = encoderFactory;
        }

    
        @Override
        public void addDashboardEntries(ShuffleboardContainer container, ControllerImplementation controller) {
            EnclosedSteerControllerFactory.super.addDashboardEntries(container, controller);
            container.addNumber("Absolute Encoder Angle", () -> Math.toDegrees(controller.getStateAngle()));
        }
   

        @Override
        public ControllerImplementation create(TalonSteerConfiguration<T> steerConfiguration, ModuleConfiguration moduleConfiguration) {
           
            TalonSRXFeedbackDevice feedback = steerConfiguration.getEncoderConfiguration();
            double offset = steerConfiguration.getOffset(); 
            final double sensorPositionCoefficient = 2.0 * Math.PI / TICKS_PER_ROTATION * moduleConfiguration.getSteerReduction();
            final double sensorVelocityCoefficient = sensorPositionCoefficient * 10.0;
            
            
            

            TalonSRXConfiguration motorConfiguration = new TalonSRXConfiguration();
            if (hasPidConstants()) {
                motorConfiguration.slot0.kP = proportionalConstant;
                motorConfiguration.slot0.kI = integralConstant;
                motorConfiguration.slot0.kD = derivativeConstant;
            }
            if (hasMotionMagic()) {
                if (hasVoltageCompensation()) {
                    motorConfiguration.slot0.kF = (1023.0 * sensorVelocityCoefficient / nominalVoltage) * velocityConstant;
                }
                // TODO: What should be done if no nominal voltage is configured? Use a default voltage?

                // TODO: Make motion magic max voltages configurable or dynamically determine optimal values
                motorConfiguration.motionCruiseVelocity = 2.0 / velocityConstant / sensorVelocityCoefficient;
                motorConfiguration.motionAcceleration = (8.0 - 2.0) / accelerationConstant / sensorVelocityCoefficient;
            }
            if (hasVoltageCompensation()) {
                motorConfiguration.voltageCompSaturation = nominalVoltage;
            }
            if (hasCurrentLimit()) {
                
            }

            WPI_TalonSRX motor = new WPI_TalonSRX(steerConfiguration.getMotorPort());
            motor.configAllSettings(motorConfiguration, CAN_TIMEOUT_MS);

            if (hasVoltageCompensation()) {
                motor.enableVoltageCompensation(true);

            
            }
           
            switch(feedback){
                default: 
                motor.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.Analog, 0, CAN_TIMEOUT_MS);
                case Analog: 
                motor.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.Analog, 0, CAN_TIMEOUT_MS);
                break; 
                case  CTRE_MagEncoder_Absolute: 
                motor.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.CTRE_MagEncoder_Relative, 0, CAN_TIMEOUT_MS);
                motor.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.CTRE_MagEncoder_Absolute, 1, CAN_TIMEOUT_MS);


            }
                       
            switch(feedback){
                default: 
                motor.setSelectedSensorPosition(Math.toRadians(motor.getSelectedSensorPosition(0))- offset);
                case Analog: 
                motor.setSelectedSensorPosition(Math.toRadians(motor.getSelectedSensorPosition(0))- offset);
                break; 
                case  CTRE_MagEncoder_Absolute: 
                motor.setSelectedSensorPosition(motor.getSelectedSensorPosition(0)*sensorPositionCoefficient- offset);
                break;

            }
            

            

            motor.setSensorPhase(true);
            motor.setInverted(moduleConfiguration.isSteerInverted() ? InvertType.InvertMotorOutput : InvertType.None);
            motor.setNeutralMode(NeutralMode.Brake);

            

            // Reduce CAN status frame rates on real robots
            // Don't do this in simulation, or it causes lag and quantization of the voltage
            // signals which cause the sim model to be inaccurate and unstable.
            motor.setStatusFramePeriod(
                    StatusFrameEnhanced.Status_1_General,
                    RobotBase.isSimulation()?20:STATUS_FRAME_GENERAL_PERIOD_MS,
                    CAN_TIMEOUT_MS
            );

            return new ControllerImplementation(motor,
                    sensorPositionCoefficient,
                    sensorVelocityCoefficient,
                    hasMotionMagic() ? TalonSRXControlMode.MotionMagic : TalonSRXControlMode.Position, DCMotor.getVex775Pro(1), feedback
                    );
        }
    }

    private static class ControllerImplementation implements EnclosedSteerController {
        private final WPI_TalonSRX motor;
        private final double motorEncoderPositionCoefficient;
        private final TalonSRXControlMode motorControlMode;
        private final TalonSRXFeedbackDevice sensor;
        private final DCMotor motorType; 


    

        private double referenceAngleRadians = 0.0;

        private ControllerImplementation(WPI_TalonSRX motor,
                                         double motorEncoderPositionCoefficient,
                                         double motorEncoderVelocityCoefficient,
                                         TalonSRXControlMode motorControlMode, DCMotor motorType, TalonSRXFeedbackDevice sensor
                                         ) {
            this.motor = motor;
            this.motorEncoderPositionCoefficient = motorEncoderPositionCoefficient;
            this.motorControlMode = motorControlMode;
            this.motorType = motorType;
            this.sensor = sensor;
            
        }

        @Override
        public double getReferenceAngle() {
            return referenceAngleRadians;
        }

        @Override
        public void setReferenceAngle(double referenceAngleRadians) {

            double currentAngleRadians;

            switch(sensor){
                default: 
                currentAngleRadians = Math.toRadians(motor.getSelectedSensorPosition());
                case Analog: 
                currentAngleRadians = Math.toRadians(motor.getSelectedSensorPosition());
                break; 
                case CTRE_MagEncoder_Absolute:
                currentAngleRadians = motor.getSelectedSensorPosition(0) * motorEncoderPositionCoefficient;


                break;


            }


            

            double currentAngleRadiansMod = currentAngleRadians % (2.0 * Math.PI);
            if (currentAngleRadiansMod < 0.0) {
                currentAngleRadiansMod += 2.0 * Math.PI;
            }

            // The reference angle has the range [0, 2pi) but the Falcon's encoder can go above that
            double adjustedReferenceAngleRadians = referenceAngleRadians + currentAngleRadians - currentAngleRadiansMod;
            if (referenceAngleRadians - currentAngleRadiansMod > Math.PI) {
                adjustedReferenceAngleRadians -= 2.0 * Math.PI;
            } else if (referenceAngleRadians - currentAngleRadiansMod < -Math.PI) {
                adjustedReferenceAngleRadians += 2.0 * Math.PI;
            }

            motor.set(motorControlMode, adjustedReferenceAngleRadians / motorEncoderPositionCoefficient);


            this.referenceAngleRadians = referenceAngleRadians;
        }

        @Override
        public void setSteerEncoder(double position, double velocity) {
            // Position is in revolutions.  Velocity is in RPM
            // TalonFX wants steps for postion.  Steps per 100ms for velocity.  Falcon integrated encoder has 2048 CPR.
           
        }

        @Override
        public double getStateAngle() {
            
       

            double motorAngleRadians;
            switch(this.sensor){
                default: 
                motorAngleRadians = motor.getSelectedSensorPosition();
                case Analog: 
                motorAngleRadians = Math.toRadians(motor.getSelectedSensorPosition());
                break; 
                case CTRE_MagEncoder_Absolute:
                motorAngleRadians = motor.getSelectedSensorPosition() * motorEncoderPositionCoefficient;
                break;


            }
            motorAngleRadians %= 2.0 * Math.PI;
            if (motorAngleRadians < 0.0) {
                motorAngleRadians += 2.0 * Math.PI;
            }

            return motorAngleRadians;
        }

        @Override
        public DCMotor getSteerMotor() {
            return motorType;
        }

       
        @Override
        public double getOutputVoltage() {
            return motor.getMotorOutputVoltage();
        }

  

        
    }
}
    
 



    





    
    


