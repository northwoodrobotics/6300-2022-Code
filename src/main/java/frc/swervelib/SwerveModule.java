package frc.swervelib;

public interface SwerveModule {
    double getDriveVelocity();

    double getSteerAngle();

    ModuleConfiguration getModuleConfiguration();

    DriveController getDriveController();

    SteerController getSteerController();

    AbsoluteEncoder getAbsoluteEncoder();
    

    void resetWheelEncoder();
    void setVelocity(double driveVelocity, double steerAngle);

    void set(double driveVoltage, double steerAngle);
}
