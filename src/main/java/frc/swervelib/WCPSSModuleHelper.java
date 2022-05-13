package frc.swervelib;
import frc.swervelib.ctre.*;
import frc.swervelib.rev.NeoDriveControllerFactoryBuilder;
import frc.swervelib.rev.NeoSteerConfiguration;
import frc.swervelib.rev.NeoSteerControllerFactoryBuilder;
import frc.wpiClasses.QuadSwerveSim;
import frc.wpiClasses.SwerveModuleSim;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;

public class WCPSSModuleHelper {
    private WCPSSModuleHelper(){

    }
    private static DriveControllerFactory<?, Integer> getFalcon500DriveFactory(WCPSSModuleConfiguration configuration) {
        return new Falcon500DriveControllerFactoryBuilder()
                .withVoltageCompensation(configuration.getNominalVoltage())
                .withCurrentLimit(configuration.getDriveCurrentLimit())
                .build();
    }
    private static SteerControllerFactory<?, Falcon500SteerConfiguration<CanCoderAbsoluteConfiguration>> getFalcon500SteerFactory(WCPSSModuleConfiguration configuration) {
        return new Falcon500SteerControllerFactoryBuilder()
                .withVoltageCompensation(configuration.getNominalVoltage())
                .withPidConstants(0.2, 0.0, 0.1)
                .withCurrentLimit(configuration.getSteerCurrentLimit())
                .build(new CanCoderFactoryBuilder()
                        .withReadingUpdatePeriod(100)
                        .build());
    }
    private static DriveControllerFactory<?, Integer> getNeoDriveFactory(WCPSSModuleConfiguration configuration) {
        return new NeoDriveControllerFactoryBuilder()
                .withVoltageCompensation(configuration.getNominalVoltage())
                .withCurrentLimit(configuration.getDriveCurrentLimit())
                .build();
    }

    private static SteerControllerFactory<?, NeoSteerConfiguration<CanCoderAbsoluteConfiguration>> getNeoSteerFactory(WCPSSModuleConfiguration configuration) {
        return new NeoSteerControllerFactoryBuilder()
                .withVoltageCompensation(configuration.getNominalVoltage())
                .withPidConstants(1.0, 0.0, 0.1)
                .withCurrentLimit(configuration.getSteerCurrentLimit())
                .build(new CanCoderFactoryBuilder()
                        .withReadingUpdatePeriod(100)
                        .build());
    }
    public enum GearRatio {
        SLOW(WCPModuleConfiguration.WCPSS_SLOW),
        SLOWER(WCPModuleConfiguration.WCPSS_SLOWER),
        STANDARD(WCPModuleConfiguration.WCPSS_STANDARD);
        

        private final ModuleConfiguration configuration;

        GearRatio(ModuleConfiguration configuration) {
            this.configuration = configuration;
        }

        public ModuleConfiguration getConfiguration() {
            return configuration;
        }
    }

     /**
     * Creates a WCP SS swerve module that uses Falcon 500s for driving and steering.
     * Module information is displayed in the specified ShuffleBoard container.
     *
     * @param container        The container to display module information in.
     * @param configuration    Module configuration parameters to use.
     * @param gearRatio        The gearing configuration the module is in.
     * @param driveMotorPort   The CAN ID of the drive Falcon 500.
     * @param steerMotorPort   The CAN ID of the steer Falcon 500.
     * @param steerEncoderPort The CAN ID of the steer CANCoder.
     * @param steerOffset      The offset of the CANCoder in radians.
     * @param namePrefix       The name of the swerve module for unique identification
     * @return The configured swerve module.
     */
    public static SwerveModule createFalcon500(
            ShuffleboardLayout container,
            WCPSSModuleConfiguration configuration,
            GearRatio gearRatio,
            int driveMotorPort,
            int steerMotorPort,
            int steerEncoderPort,
            double steerOffset,
            String namePrefix
    ) {
        return new SwerveModuleFactory<>(
                gearRatio.getConfiguration(),
                getFalcon500DriveFactory(configuration),
                getFalcon500SteerFactory(configuration)
        ).create(
                container,
                driveMotorPort,
                new Falcon500SteerConfiguration<>(
                        steerMotorPort,
                        new CanCoderAbsoluteConfiguration(steerEncoderPort, steerOffset)
                ), namePrefix
                
        );
    }
    
     /**
     * Creates a WCP SS swerve module that uses Falcon 500s for driving and steering.
     * Module information is displayed in the specified ShuffleBoard container.
     *
     * @param container        The container to display module information in.
     * @param gearRatio        The gearing configuration the module is in.
     * @param driveMotorPort   The CAN ID of the drive Falcon 500.
     * @param steerMotorPort   The CAN ID of the steer Falcon 500.
     * @param steerEncoderPort The CAN ID of the steer CANCoder.
     * @param steerOffset      The offset of the CANCoder in radians.
     * @param namePrefix       The name of the swerve module for unique identification
     * @return The configured swerve module.
     */
    public static SwerveModule createFalcon500(
            ShuffleboardLayout container,
            GearRatio gearRatio,
            int driveMotorPort,
            int steerMotorPort,
            int steerEncoderPort,
            double steerOffset, String namePrefix
    ) {
        return createFalcon500(container, new WCPSSModuleConfiguration(), gearRatio, driveMotorPort, steerMotorPort, steerEncoderPort, steerOffset, namePrefix);
    }
    
      /**
     * Creates a Mk4 swerve module that uses Falcon 500s for driving and steering.
     *
     * @param configuration    Module configuration parameters to use.
     * @param gearRatio        The gearing configuration the module is in.
     * @param driveMotorPort   The CAN ID of the drive Falcon 500.
     * @param steerMotorPort   The CAN ID of the steer Falcon 500.
     * @param steerEncoderPort The CAN ID of the steer CANCoder.
     * @param steerOffset      The offset of the CANCoder in radians.
     * @param namePrefix       The name of the swerve module for unique identification
     * @return The configured swerve module.
     */
    public static SwerveModule createFalcon500(
            WCPSSModuleConfiguration configuration,
            GearRatio gearRatio,
            int driveMotorPort,
            int steerMotorPort,
            int steerEncoderPort,
            double steerOffset, String namePrefix
    ) {
        return new SwerveModuleFactory<>(
                gearRatio.getConfiguration(),
                getFalcon500DriveFactory(configuration),
                getFalcon500SteerFactory(configuration)
        ).create(
                driveMotorPort,
                new Falcon500SteerConfiguration<>(
                        steerMotorPort,
                        new CanCoderAbsoluteConfiguration(steerEncoderPort, steerOffset)
                ), namePrefix
        );
    }
      /**
     * Creates a WCP SS swerve module that uses Falcon 500s for driving and steering.
     *
     * @param gearRatio        The gearing configuration the module is in.
     * @param driveMotorPort   The CAN ID of the drive Falcon 500.
     * @param steerMotorPort   The CAN ID of the steer Falcon 500.
     * @param steerEncoderPort The CAN ID of the steer CANCoder.
     * @param steerOffset      The offset of the CANCoder in radians.
     * @param namePrefix       The name of the swerve module for unique identification
     * @return The configured swerve module.
     */
    public static SwerveModule createFalcon500(
            GearRatio gearRatio,
            int driveMotorPort,
            int steerMotorPort,
            int steerEncoderPort,
            double steerOffset, String namePrefix
    ) {
        return createFalcon500(new WCPSSModuleConfiguration(), gearRatio, driveMotorPort, steerMotorPort, steerEncoderPort, steerOffset, namePrefix);
    }
    /**
     * Creates a WCP SS swerve module that uses NEOs for driving and steering.
     * Module information is displayed in the specified ShuffleBoard container.
     *
     * @param container        The container to display module information in.
     * @param configuration    Module configuration parameters to use.
     * @param gearRatio        The gearing configuration the module is in.
     * @param driveMotorPort   The CAN ID of the drive NEO.
     * @param steerMotorPort   The CAN ID of the steer NEO.
     * @param steerEncoderPort The CAN ID of the steer CANCoder.
     * @param steerOffset      The offset of the CANCoder in radians.
     * @param namePrefix       The name of the swerve module for unique identification
     * @return The configured swerve module.
     */
    public static SwerveModule createNeo(
            ShuffleboardLayout container,
            WCPSSModuleConfiguration configuration,
            GearRatio gearRatio,
            int driveMotorPort,
            int steerMotorPort,
            int steerEncoderPort,
            double steerOffset, String namePrefix
    ) {
        return new SwerveModuleFactory<>(
                gearRatio.getConfiguration(),
                getNeoDriveFactory(configuration),
                getNeoSteerFactory(configuration)
        ).create(
                container,
                driveMotorPort,
                new NeoSteerConfiguration<>(
                        steerMotorPort,
                        new CanCoderAbsoluteConfiguration(steerEncoderPort, steerOffset)
                ),namePrefix
        );
    }

    /**
     * Creates a WCP SS swerve module that uses NEOs for driving and steering.
     * Module information is displayed in the specified ShuffleBoard container.
     *
     * @param container        The container to display module information in.
     * @param gearRatio        The gearing configuration the module is in.
     * @param driveMotorPort   The CAN ID of the drive NEO.
     * @param steerMotorPort   The CAN ID of the steer NEO.
     * @param steerEncoderPort The CAN ID of the steer CANCoder.
     * @param steerOffset      The offset of the CANCoder in radians.
     * @param namePrefix       The name of the swerve module for unique identification
     * @return The configured swerve module.
     */
    public static SwerveModule createNeo(
            ShuffleboardLayout container,
            GearRatio gearRatio,
            int driveMotorPort,
            int steerMotorPort,
            int steerEncoderPort,
            double steerOffset, String namePrefix
    ) {
        return createNeo(container, new WCPSSModuleConfiguration(), gearRatio, driveMotorPort, steerMotorPort, steerEncoderPort, steerOffset, namePrefix);
    }

     /**
     * Creates a WCP SS swerve module that uses NEOs for driving and steering.
     *
     * @param configuration    Module configuration parameters to use.
     * @param gearRatio        The gearing configuration the module is in.
     * @param driveMotorPort   The CAN ID of the drive NEO.
     * @param steerMotorPort   The CAN ID of the steer NEO.
     * @param steerEncoderPort The CAN ID of the steer CANCoder.
     * @param steerOffset      The offset of the CANCoder in radians.
     * @param namePrefix       The name of the swerve module for unique identification
     * @return The configured swerve module.
     */
    public static SwerveModule createNeo(
            WCPSSModuleConfiguration configuration,
            GearRatio gearRatio,
            int driveMotorPort,
            int steerMotorPort,
            int steerEncoderPort,
            double steerOffset, String namePrefix
    ) {
        return new SwerveModuleFactory<>(
                gearRatio.getConfiguration(),
                getNeoDriveFactory(configuration),
                getNeoSteerFactory(configuration)
        ).create(
                driveMotorPort,
                new NeoSteerConfiguration<>(
                        steerMotorPort,
                        new CanCoderAbsoluteConfiguration(steerEncoderPort, steerOffset)
                ), namePrefix
        );
    }

    /**
     * Creates a WCP SS swerve module that uses NEOs for driving and steering.
     *
     * @param gearRatio        The gearing configuration the module is in.
     * @param driveMotorPort   The CAN ID of the drive NEO.
     * @param steerMotorPort   The CAN ID of the steer NEO.
     * @param steerEncoderPort The CAN ID of the steer CANCoder.
     * @param steerOffset      The offset of the CANCoder in radians.
     * @param namePrefix       The name of the swerve module for unique identification
     * @return The configured swerve module.
     */
    public static SwerveModule createNeo(
            GearRatio gearRatio,
            int driveMotorPort,
            int steerMotorPort,
            int steerEncoderPort,
            double steerOffset, String namePrefix
    ) {
        return createNeo(new WCPSSModuleConfiguration(), gearRatio, driveMotorPort, steerMotorPort, steerEncoderPort, steerOffset,  namePrefix);
    }

     /**
     * Creates a WCP SS swerve module that uses a Falcon 500 for driving and a NEO for steering.
     * Module information is displayed in the specified ShuffleBoard container.
     *
     * @param container        The container to display module information in.
     * @param configuration    Module configuration parameters to use.
     * @param gearRatio        The gearing configuration the module is in.
     * @param driveMotorPort   The CAN ID of the drive Falcon 500.
     * @param steerMotorPort   The CAN ID of the steer NEO.
     * @param steerEncoderPort The CAN ID of the steer CANCoder.
     * @param steerOffset      The offset of the CANCoder in radians.
     * @param namePrefix       The name of the swerve module for unique identification
     * @return The configured swerve module.
     */
    public static SwerveModule createFalcon500Neo(
            ShuffleboardLayout container,
            WCPSSModuleConfiguration configuration,
            GearRatio gearRatio,
            int driveMotorPort,
            int steerMotorPort,
            int steerEncoderPort,
            double steerOffset, String namePrefix
    ) {
        return new SwerveModuleFactory<>(
                gearRatio.getConfiguration(),
                getFalcon500DriveFactory(configuration),
                getNeoSteerFactory(configuration)
        ).create(
                container,
                driveMotorPort,
                new NeoSteerConfiguration<>(
                        steerMotorPort,
                        new CanCoderAbsoluteConfiguration(steerEncoderPort, steerOffset)
                ), namePrefix
        );
    }

    /**
     * Creates a WCP SS swerve module that uses a Falcon 500 for driving and a NEO for steering.
     * Module information is displayed in the specified ShuffleBoard container.
     *
     * @param container        The container to display module information in.
     * @param gearRatio        The gearing configuration the module is in.
     * @param driveMotorPort   The CAN ID of the drive Falcon 500.
     * @param steerMotorPort   The CAN ID of the steer NEO.
     * @param steerEncoderPort The CAN ID of the steer CANCoder.
     * @param steerOffset      The offset of the CANCoder in radians.
     * @param namePrefix       The name of the swerve module for unique identification
     * @return The configured swerve module.
     */
    public static SwerveModule createFalcon500Neo(
            ShuffleboardLayout container,
            GearRatio gearRatio,
            int driveMotorPort,
            int steerMotorPort,
            int steerEncoderPort,
            double steerOffset, String namePrefix
    ) {
        return createFalcon500Neo(container, new WCPSSModuleConfiguration(), gearRatio, driveMotorPort, steerMotorPort, steerEncoderPort, steerOffset, namePrefix);
    }

     /**
     * Creates a WCP SS swerve module that uses a Falcon 500 for driving and a NEO for steering.
     *
     * @param configuration    Module configuration parameters to use.
     * @param gearRatio        The gearing configuration the module is in.
     * @param driveMotorPort   The CAN ID of the drive Falcon 500.
     * @param steerMotorPort   The CAN ID of the steer NEO.
     * @param steerEncoderPort The CAN ID of the steer CANCoder.
     * @param steerOffset      The offset of the CANCoder in radians.
     * @param namePrefix       The name of the swerve module for unique identification
     * @return The configured swerve module.
     */
    public static SwerveModule createFalcon500Neo(
            WCPSSModuleConfiguration configuration,
            GearRatio gearRatio,
            int driveMotorPort,
            int steerMotorPort,
            int steerEncoderPort,
            double steerOffset, String namePrefix
    ) {
        return new SwerveModuleFactory<>(
                gearRatio.getConfiguration(),
                getFalcon500DriveFactory(configuration),
                getNeoSteerFactory(configuration)
        ).create(
                driveMotorPort,
                new NeoSteerConfiguration<>(
                        steerMotorPort,
                        new CanCoderAbsoluteConfiguration(steerEncoderPort, steerOffset)
                ), namePrefix
        );
    }

    /**
     * Creates a WCP SS swerve module that uses a Falcon 500 for driving and a NEO for steering.
     *
     * @param gearRatio        The gearing configuration the module is in.
     * @param driveMotorPort   The CAN ID of the drive Falcon 500.
     * @param steerMotorPort   The CAN ID of the steer NEO.
     * @param steerEncoderPort The CAN ID of the steer CANCoder.
     * @param steerOffset      The offset of the CANCoder in radians.
     * @param namePrefix       The name of the swerve module for unique identification
     * @return The configured swerve module.
     */
    public static SwerveModule createFalcon500Neo(
            GearRatio gearRatio,
            int driveMotorPort,
            int steerMotorPort,
            int steerEncoderPort,
            double steerOffset, String namePrefix
    ) {
        return createFalcon500Neo(new WCPSSModuleConfiguration(), gearRatio, driveMotorPort, steerMotorPort, steerEncoderPort, steerOffset, namePrefix);
    }

    /**
     * Creates a WCP SS swerve module that uses a NEO for driving and a Falcon 500 for steering.
     * Module information is displayed in the specified ShuffleBoard container.
     *
     * @param container        The container to display module information in.
     * @param configuration    Module configuration parameters to use.
     * @param gearRatio        The gearing configuration the module is in.
     * @param driveMotorPort   The CAN ID of the drive NEO.
     * @param steerMotorPort   The CAN ID of the steer Falcon 500.
     * @param steerEncoderPort The CAN ID of the steer CANCoder.
     * @param steerOffset      The offset of the CANCoder in radians.
     * @param namePrefix       The name of the swerve module for unique identification
     * @return The configured swerve module.
     */
    public static SwerveModule createNeoFalcon500(
            ShuffleboardLayout container,
            WCPSSModuleConfiguration configuration,
            GearRatio gearRatio,
            int driveMotorPort,
            int steerMotorPort,
            int steerEncoderPort,
            double steerOffset, String namePrefix
    ) {
        return new SwerveModuleFactory<>(
                gearRatio.getConfiguration(),
                getNeoDriveFactory(configuration),
                getFalcon500SteerFactory(configuration)
        ).create(
                container,
                driveMotorPort,
                new Falcon500SteerConfiguration<>(
                        steerMotorPort,
                        new CanCoderAbsoluteConfiguration(steerEncoderPort, steerOffset)
                ), namePrefix
        );
    }

    /**
     * Creates a Mk4 swerve module that uses a NEO for driving and a Falcon 500 for steering.
     * Module information is displayed in the specified ShuffleBoard container.
     *
     * @param container        The container to display module information in.
     * @param gearRatio        The gearing configuration the module is in.
     * @param driveMotorPort   The CAN ID of the drive NEO.
     * @param steerMotorPort   The CAN ID of the steer Falcon 500.
     * @param steerEncoderPort The CAN ID of the steer CANCoder.
     * @param steerOffset      The offset of the CANCoder in radians.
     * @param namePrefix       The name of the swerve module for unique identification
     * @return The configured swerve module.
     */
    public static SwerveModule createNeoFalcon500(
            ShuffleboardLayout container,
            GearRatio gearRatio,
            int driveMotorPort,
            int steerMotorPort,
            int steerEncoderPort,
            double steerOffset, String namePrefix
    ) {
        return createNeoFalcon500(container, new WCPSSModuleConfiguration(), gearRatio, driveMotorPort, steerMotorPort, steerEncoderPort, steerOffset, namePrefix);
    }

    /**
     * Creates a WCP swerve module that uses a NEO for driving and a Falcon 500 for steering.
     *
     * @param configuration    Module configuration parameters to use.
     * @param gearRatio        The gearing configuration the module is in.
     * @param driveMotorPort   The CAN ID of the drive NEO.
     * @param steerMotorPort   The CAN ID of the steer Falcon 500.
     * @param steerEncoderPort The CAN ID of the steer CANCoder.
     * @param steerOffset      The offset of the CANCoder in radians.
     * @param namePrefix       The name of the swerve module for unique identification
     * @return The configured swerve module.
     */
    public static SwerveModule createNeoFalcon500(
            WCPSSModuleConfiguration configuration,
            GearRatio gearRatio,
            int driveMotorPort,
            int steerMotorPort,
            int steerEncoderPort,
            double steerOffset, String namePrefix
    ) {
        return new SwerveModuleFactory<>(
                gearRatio.getConfiguration(),
                getNeoDriveFactory(configuration),
                getFalcon500SteerFactory(configuration)
        ).create(
                driveMotorPort,
                new Falcon500SteerConfiguration<>(
                        steerMotorPort,
                        new CanCoderAbsoluteConfiguration(steerEncoderPort, steerOffset)
                ), namePrefix
        );
    }

    /**
     * Creates a Mk4 swerve module that uses a NEO for driving and a Falcon 500 for steering.
     *
     * @param gearRatio        The gearing configuration the module is in.
     * @param driveMotorPort   The CAN ID of the drive NEO.
     * @param steerMotorPort   The CAN ID of the steer Falcon 500.
     * @param steerEncoderPort The CAN ID of the steer CANCoder.
     * @param steerOffset      The offset of the CANCoder in radians.
     * @param namePrefix       The name of the swerve module for unique identification
     * @return The configured swerve module.
     */
    public static SwerveModule createNeoFalcon500(
            GearRatio gearRatio,
            int driveMotorPort,
            int steerMotorPort,
            int steerEncoderPort,
            double steerOffset, String namePrefix
    ) {
        return createNeoFalcon500(new WCPSSModuleConfiguration(), gearRatio, driveMotorPort, steerMotorPort, steerEncoderPort, steerOffset, namePrefix);
    }

    public static SwerveModuleSim createSim(SwerveModule module) {
        ModuleConfiguration modConfig = module.getModuleConfiguration();
        return new SwerveModuleSim(module.getSteerController().getSteerMotor(),
                                   module.getDriveController().getDriveMotor(),
                                   modConfig.getWheelDiameter() / 2,
                                   1 / modConfig.getSteerReduction(),
                                   1 / modConfig.getDriveReduction(),
                                   1.0, // CANCoder is directly on the shaft
                                   1 / modConfig.getDriveReduction(),
                                   1.1,
                                   0.8,
                                   SwerveConstants.MASS_kg * 9.81 / QuadSwerveSim.NUM_MODULES, 
                                   0.01
                                   );
    }







    


}
