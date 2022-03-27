package frc.swervelib;

import edu.wpi.first.math.system.plant.DCMotor;

public interface DriveController {
    void setReferenceVoltage(double voltage);

    void resetEncoder();

    DCMotor getDriveMotor();

    double getStateVelocity();
    

    double getOutputVoltage();
    void setVelocity(double velocity);

    void setDriveEncoder(double position, double velocity);
}
