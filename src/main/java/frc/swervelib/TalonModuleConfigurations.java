package frc.swervelib;

public final class TalonModuleConfigurations {
    public static final ModuleConfiguration SwerveAndSteer = new ModuleConfiguration(
            0.1016,
            (12.0 / 40.0) * (20.0 / 40.0),
            true,
            (1/1) * (1/1),
            true
    );

    public static final ModuleConfiguration ThriftySwerve = new ModuleConfiguration(
        0.0762, 
        (12/21) * (15/45), 
        false, 
        1/1, 
        false);
    
}
