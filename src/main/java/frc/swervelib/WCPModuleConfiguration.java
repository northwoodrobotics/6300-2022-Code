package frc.swervelib;

public final class WCPModuleConfiguration {
    public static final ModuleConfiguration WCPSS_STANDARD = new ModuleConfiguration(
        0.1016,
        (10/40)* (30/18)*(15/45),
        true, 
        (8/24)* (18/72),
        true

    );

    public static final ModuleConfiguration WCPSS_SLOWER = new ModuleConfiguration(
        0.1016,
        (12/40)* (26/22)*(15/45),
        true, 
        (8/24)* (18/72),
        true

    );
    public static final ModuleConfiguration WCPSS_SLOW = new ModuleConfiguration(
        0.1016,
        (11/40)* (26/22)*(15/45),
        true, 
        (8/24)* (18/72),
        true

    );

    public static final ModuleConfiguration SWERVEX_SLOW = new ModuleConfiguration(
        0.1016,
        (10/34)* (26/20)*(15/45),
        true, 
        (8/24)* (14/72),
        true

    );
    public static final ModuleConfiguration SWERVEX_STANDARD = new ModuleConfiguration(
        0.1016,
        (11/34)* (26/20)*(15/45),
        true, 
        (8/24)* (14/72),
        true

    );
    public static final ModuleConfiguration SWERVEX_FAST= new ModuleConfiguration(
        0.1016,
        (12/34)* (26/20)*(15/45),
        true, 
        (8/24)* (14/72),
        true

    );
    public static final ModuleConfiguration SWERVEX_FlIPPED_FAST= new ModuleConfiguration(
        0.1016,
        (12/24)* (24/22)*(15/45),
        true, 
        (12/24)* (14/72),
        true

    );
    public static final ModuleConfiguration SWERVEX_FlIPPED_STANDARD= new ModuleConfiguration(
        0.1016,
        (12/24)* (22/24)*(15/45),
        true, 
        (12/24)* (14/72),
        true

    );
    public static final ModuleConfiguration SWERVEX_FlIPPED_SLOW= new ModuleConfiguration(
        0.1016,
        (12/24)* (22/26)*(15/45),
        true, 
        (12/24)* (14/72),
        true

    );





    

     
}
