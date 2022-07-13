package frc.swervelib;

import frc.swervelib.ctre.*;
import frc.swervelib.rev.NeoDriveControllerFactoryBuilder;
import frc.wpiClasses.QuadSwerveSim;
import frc.wpiClasses.SwerveModuleSim;

import com.ctre.phoenix.motorcontrol.TalonSRXFeedbackDevice;

import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;

public final  class SwerveTalonModuleHelper {
    private SwerveTalonModuleHelper(){

    }

    private static TalonSRXFeedbackDevice SwerveAndSteerDevice = TalonSRXFeedbackDevice.Analog;
    private static TalonSRXFeedbackDevice TBDevice = TalonSRXFeedbackDevice.CTRE_MagEncoder_Absolute;
    private static GearRatio SwerveAndSteerRatio = GearRatio.SS; 
    private static GearRatio ThirftyRatio = GearRatio.TS; 

    private static DriveControllerFactory<?, Integer> getFalcon500DriveFactory(TalonSteerModuleConfiguration configuration) {
        return new Falcon500DriveControllerFactoryBuilder()
                .withVoltageCompensation(configuration.getNominalVoltage())
                .withCurrentLimit(configuration.getDriveCurrentLimit())
                .build();
    }
    private static EnclosedSteerControllerFactory<?, TalonSteerConfiguration<CanCoderAbsoluteConfiguration>> getSwerveAndSteerFactory(TalonSteerModuleConfiguration configuration) {
        return new TalonSteerFactoryBuilder()
                .withVoltageCompensation(configuration.getNominalVoltage())
                .withPidConstants(10, 0.0, 0)
                .withCurrentLimit(configuration.getSteerCurrentLimit())
                .build(null);
    }
    private static EnclosedSteerControllerFactory<?, TalonSteerConfiguration<CanCoderAbsoluteConfiguration>> getThriftyFactory(TalonSteerModuleConfiguration configuration) {
        return new TalonSteerFactoryBuilder()
                .withVoltageCompensation(configuration.getNominalVoltage())
                .withPidConstants(1, 0.0, 0.01)
                .withCurrentLimit(configuration.getSteerCurrentLimit())
                .build(null);
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
            int driveMotorPort,
            int steerMotorPort,
            double steerOffset,
            String namePrefix
    ) {
        return new EnclosedEncoderModuleFactory<>(
                SwerveAndSteerRatio.getConfiguration(),
                getFalcon500DriveFactory(configuration),
                getSwerveAndSteerFactory(configuration)
        ).create(
                container,
                driveMotorPort,
                new TalonSteerConfiguration<>(
                        steerMotorPort, 
                        SwerveAndSteerDevice, 
                        null, 
                        steerOffset
                        
                ), namePrefix
                
        );
    }

       /**
     * Creates a AndyMark swerve module that uses Falcon 500s for driving and a Talon SRX for steering.
     * Module information is displayed in the specified ShuffleBoard container.
     *
     * @param container        The container to display module information in.
     * @param configuration    Module configuration parameters to use.
     * @param gearRatio        The gearing configuration the module is in.
     * @param driveMotorPort   The CAN ID of the drive Falcon 500.
     * @param steerMotorPort   The CAN ID of the steer TalonSRX 500.
     * @param steerOffset      The offset of the CANCoder in radians.
     * @param namePrefix       The name of the swerve module for unique identification
     * @return The configured swerve module.
     */
    public static SwerveModule createNEOSwerveAndSteer(
            ShuffleboardLayout container,
            TalonSteerModuleConfiguration configuration,
            int driveMotorPort,
            int steerMotorPort,
            double steerOffset,
            String namePrefix
    ) {
        return new EnclosedEncoderModuleFactory<>(
                SwerveAndSteerRatio.getConfiguration(),
                getNeoDriveFactory(configuration),
                getSwerveAndSteerFactory(configuration)
                ).create(
                container,
                driveMotorPort,
                new TalonSteerConfiguration<>(
                        steerMotorPort,SwerveAndSteerDevice,
                        null, steerOffset
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
            GearRatio gearRatio,
            int driveMotorPort,
            int steerMotorPort,
            double steerOffset,
            String namePrefix
    ) {
        return createFalconSwerveAndSteer(container, new TalonSteerModuleConfiguration(), driveMotorPort, steerMotorPort, steerOffset, namePrefix);
    }

          /**
     * Creates a Thrifty swerve module that uses Falcon 500s for driving and steering.
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


    public static SwerveModule createFalconThriftySwerve(
        ShuffleboardLayout container,
        TalonSteerModuleConfiguration configuration,        
        int driveMotorPort,
        int steerMotorPort,
        double steerOffset,
        String namePrefix
) {
    return new EnclosedEncoderModuleFactory<>(
            ThirftyRatio.getConfiguration(),
            getNeoDriveFactory(configuration),
            getThriftyFactory(configuration)
            ).create(
            container,
            driveMotorPort,
            new TalonSteerConfiguration<>(
                    steerMotorPort,TBDevice, 
                    null, steerOffset
            ), namePrefix
            
    );

}
        public static SwerveModule createFalconThriftySwerve(
        ShuffleboardLayout container,
        GearRatio gearRatio,
        int driveMotorPort,
        int steerMotorPort,
        double steerOffset,
        String namePrefix
        ) {
        return createFalconThriftySwerve(container, new TalonSteerModuleConfiguration(), driveMotorPort, steerMotorPort, steerOffset, namePrefix);
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

