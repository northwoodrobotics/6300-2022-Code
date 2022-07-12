package frc.swervelib;

import frc.swervelib.ctre.*;
import frc.swervelib.rev.NeoDriveControllerFactoryBuilder;
import frc.wpiClasses.QuadSwerveSim;
import frc.wpiClasses.SwerveModuleSim;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;

public final  class SwerveTalonModuleHelper {
    private SwerveTalonModuleHelper(){

    }

    private static DriveControllerFactory<?, Integer> getFalcon500DriveFactory(TalonSteerModuleConfiguration configuration) {
        return new Falcon500DriveControllerFactoryBuilder()
                .withVoltageCompensation(configuration.getNominalVoltage())
                .withCurrentLimit(configuration.getDriveCurrentLimit())
                .build();
    }
    private static EnclosedSteerControllerFactory<?, TalonSteerConfiguration<CanCoderAbsoluteConfiguration>> getSwerveAndSteerFactory(TalonSteerModuleConfiguration configuration) {
        return new TalonSteerFactoryBuilder()
                .withVoltageCompensation(configuration.getNominalVoltage())
                .withPidConstants(0.2, 0.0, 0.1)
                .withCurrentLimit(configuration.getSteerCurrentLimit())
                .build(new CanCoderFactoryBuilder()
                        .withReadingUpdatePeriod(100)
                        .build());
    }
    private static EnclosedSteerControllerFactory<?, TalonSteerConfiguration<CanCoderAbsoluteConfiguration>> getThriftyFactory(TalonSteerModuleConfiguration configuration) {
        return new TalonSteerFactoryBuilder()
                .withVoltageCompensation(configuration.getNominalVoltage())
                .withPidConstants(0.2, 0.0, 0.1)
                .withCurrentLimit(configuration.getSteerCurrentLimit())
                .build(new CanCoderFactoryBuilder()
                        .withReadingUpdatePeriod(100)
                        .build());
    }
    private static DriveControllerFactory<?, Integer> getNeoDriveFactory(TalonSteerModuleConfiguration configuration) {
        return new NeoDriveControllerFactoryBuilder()
                .withVoltageCompensation(configuration.getNominalVoltage())
                .withCurrentLimit(configuration.getDriveCurrentLimit())
                .build();
    }

    /**
     * Creates a AndyMark swerve module that uses Falcon 500s for driving and steering.
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
    public static SwerveModule createFalconSwerveAndSteer(
            ShuffleboardLayout container,
            TalonSteerModuleConfiguration configuration,
            GearRatio gearRatio,
            int driveMotorPort,
            int steerMotorPort,
            int steerEncoderPort,
            double steerOffset,
            String namePrefix
    ) {
        return new EnclosedEncoderModuleFactory<>(
                gearRatio.getConfiguration(),
                getFalcon500DriveFactory(configuration),
                getSwerveAndSteerFactory(configuration)
        ).create(
                container,
                driveMotorPort,
                new TalonSteerConfiguration<>(
                        steerMotorPort,
                        null
                ), namePrefix
                
        );
    }

       /**
     * Creates a AndyMark swerve module that uses Falcon 500s for driving and steering.
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
    public static SwerveModule createNEOSwerveAndSteer(
            ShuffleboardLayout container,
            TalonSteerModuleConfiguration configuration,
            GearRatio gearRatio,
            int driveMotorPort,
            int steerMotorPort,
            int steerEncoderPort,
            double steerOffset,
            String namePrefix
    ) {
        return new EnclosedEncoderModuleFactory<>(
                gearRatio.getConfiguration(),
                getNeoDriveFactory(configuration),
                getSwerveAndSteerFactory(configuration)
        ).create(
                container,
                driveMotorPort,
                new TalonSteerConfiguration<>(
                        steerMotorPort,
                        null
                ), namePrefix
                
        );
    }

       /**
     * Creates a AndyMark swerve module that uses Falcon 500s for driving and steering.
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
    public static SwerveModule createFalcon500SwerveAndSteer(
            ShuffleboardLayout container,
            TalonSteerModuleConfiguration configuration,
            GearRatio gearRatio,
            int driveMotorPort,
            int steerMotorPort,
            int steerEncoderPort,
            double steerOffset,
            String namePrefix
    ) {
        return createFalconSwerveAndSteer(container, configuration, gearRatio, driveMotorPort, steerMotorPort, steerEncoderPort, steerOffset, namePrefix);
    }

    public static SwerveModule createFalconThriftySwerve(
        ShuffleboardLayout container,
        TalonSteerModuleConfiguration configuration,
        GearRatio gearRatio,
        int driveMotorPort,
        int steerMotorPort,
        int steerEncoderPort,
        double steerOffset,
        String namePrefix
) {
    return new EnclosedEncoderModuleFactory<>(
            gearRatio.getConfiguration(),
            getNeoDriveFactory(configuration),
            getThriftyFactory(configuration)
    ).create(
            container,
            driveMotorPort,
            new TalonSteerConfiguration<>(
                    steerMotorPort,
                    null
            ), namePrefix
            
    );
}



    


    

    





    



    public enum GearRatio {
        TS(TalonModuleConfigurations.ThriftySwerve),
        SS(TalonModuleConfigurations.SwerveAndSteer);


        private final ModuleConfiguration configuration;

        GearRatio(ModuleConfiguration configuration) {
            this.configuration = configuration;
        }

        public ModuleConfiguration getConfiguration() {
            return configuration;
        }
    }

    
    
}

