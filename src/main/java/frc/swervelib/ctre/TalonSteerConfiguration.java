package frc.swervelib.ctre;

import java.util.Objects;

public class TalonSteerConfiguration {
    private final int motorPort;
    // encoder type, as in a CTRE MAG Encoder, or a MA3 Analog Encoder
    private final EncoderType EncoderType;

    
    public TalonSteerConfiguration(int motorPort, EncoderType type ){
        this.motorPort = motorPort; 
        this.EncoderType = type;

    }
    public int getMotorPort() {
        return motorPort;
    }

    public EncoderType getEncoderConfiguration() {
        return EncoderType;
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
        return "Falcon500SteerConfiguration{" +
                "motorPort=" + motorPort +
                ", encoderConfiguration=" + EncoderType +
                '}';
    }

    public enum EncoderType{
        Analog, Digital
    }


 




    
}
