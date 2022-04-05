package frc.swervelib.ctre;

import edu.wpi.first.wpilibj.motorcontrol.Talon;

public class TalonSRXSteerConfiguration {
    private final int motorPort; 
    //private final EncoderConfiguration
    public TalonSRXSteerConfiguration(int motorPort){
        this.motorPort = motorPort;
    }
    public int getMotorPort() {
        return motorPort;
    }
    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (o == null || getClass() != o.getClass()) return false;
        //TalonSRXSteerConfiguration<?> that = (TalonSRXSteerConfiguration<?>) o;
        return true; //== that.getMotorPort();// && getEncoderConfiguration().equals(that.getEncoderConfiguration());
    }
    @Override
    public String toString() {
        return "TalonSRXSteerConfiguration{" +
                "motorPort=" + motorPort +
                ", Analog=" +
                '}';
    }

}
