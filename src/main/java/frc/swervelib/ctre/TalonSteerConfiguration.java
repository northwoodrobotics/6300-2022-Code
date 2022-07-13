package frc.swervelib.ctre;

import java.util.Objects;

import com.ctre.phoenix.motorcontrol.TalonSRXFeedbackDevice;

public class TalonSteerConfiguration<EncoderType> {
    private final int motorPort;
    // encoder type, as in a CTRE MAG Encoder, or a MA3 Analog Encoder
    private final EncoderType encoder;
    private final TalonSRXFeedbackDevice encoderType;
    private final double encoderOffset; 

    
    public TalonSteerConfiguration(int motorPort, TalonSRXFeedbackDevice type, EncoderType sensor, double offset){
        this.motorPort = motorPort; 
        this.encoderType = type;
        this.encoder = sensor;
        this.encoderOffset = offset; 


    }
    public int getMotorPort() {
        return motorPort;
    }
    public double getOffset(){
        return encoderOffset;
    }

    public TalonSRXFeedbackDevice getEncoderConfiguration() {
        return encoderType;
    }


    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (o == null || getClass() != o.getClass()) return false;
        Falcon500SteerConfiguration<?> that = (Falcon500SteerConfiguration<?>) o;
        return getMotorPort() == that.getMotorPort() && getEncoderConfiguration().equals(that.getEncoderConfiguration());
    }

    @Override
    public int hashCode() {
        return Objects.hash(getMotorPort(), 0);
    }
    @Override
    public String toString() {
        return "TalonSteerConfiguration{" +
                "motorPort=" + motorPort +
                ", encoderConfiguration=" + encoderType +
                '}';
    }



 




    
}
