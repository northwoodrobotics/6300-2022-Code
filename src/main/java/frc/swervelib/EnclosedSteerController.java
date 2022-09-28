package frc.swervelib;

import edu.wpi.first.math.system.plant.DCMotor;

public interface EnclosedSteerController{
    double getReferenceAngle();

    void setReferenceAngle(double referenceAngleRadians);

    DCMotor getSteerMotor();

    double getStateAngle();

    double getOutputVoltage();

    void setSteerEncoder(double position, double velocity);

    
}
