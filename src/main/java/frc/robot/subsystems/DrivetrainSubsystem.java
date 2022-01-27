package frc.robot.subsystems;


import frc.swervelib.Gyroscope;
import frc.swervelib.GyroscopeHelper;
import frc.swervelib.Mk4SwerveModuleHelper;
import frc.swervelib.SwerveConstants;
import frc.swervelib.SwerveModule;
import frc.swervelib.SwerveSubsystem;
import frc.swervelib.SwerveDrivetrainModel;
import frc.wpiClasses.QuadSwerveSim;
import frc.robot.Constants;
import java.util.ArrayList;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class DrivetrainSubsystem {
        public static double SwerveVelocity;
    

    public static SwerveDrivetrainModel  createSwerveModel(){
        passConstants();

        ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");
        
        ArrayList<SwerveModule> realModules = new ArrayList<SwerveModule>(QuadSwerveSim.NUM_MODULES);

        SwerveModule m_frontLeftModule = Mk4SwerveModuleHelper.createFalcon500(
                // This parameter is optional, but will allow you to see the current state of the module on the dashboard.
                tab.getLayout("Front Left Module", BuiltInLayouts.kList)
                        .withSize(2, 4)
                        .withPosition(0, 0),
                // This can either be STANDARD or FAST depending on your gear configuration
                Mk4SwerveModuleHelper.GearRatio.L2,
                // This is the ID of the drive motor
                Constants.DriveConstants.FRONT_LEFT_MODULE_DRIVE_MOTOR,
                // This is the ID of the steer motor
                Constants.DriveConstants.FRONT_LEFT_MODULE_STEER_MOTOR,
                // This is the ID of the steer encoder
                Constants.DriveConstants.FRONT_LEFT_MODULE_STEER_ENCODER,
                // This is how much the steer encoder is offset from true zero (In our case, zero is facing straight forward)
                Constants.DriveConstants.FRONT_LEFT_MODULE_STEER_OFFSET, "FL"
        );
    
        // We will do the same for the other modules
        SwerveModule m_frontRightModule = Mk4SwerveModuleHelper.createFalcon500(
                tab.getLayout("Front Right Module", BuiltInLayouts.kList)
                        .withSize(2, 4)
                        .withPosition(2, 0),
                Mk4SwerveModuleHelper.GearRatio.L2,
                Constants.DriveConstants.FRONT_RIGHT_MODULE_DRIVE_MOTOR,
                Constants.DriveConstants.FRONT_RIGHT_MODULE_STEER_MOTOR,
                Constants.DriveConstants.FRONT_RIGHT_MODULE_STEER_ENCODER,
                Constants.DriveConstants.FRONT_RIGHT_MODULE_STEER_OFFSET, "FR"
        );
    
        SwerveModule m_backLeftModule = Mk4SwerveModuleHelper.createFalcon500(
                tab.getLayout("Back Left Module", BuiltInLayouts.kList)
                        .withSize(2, 4)
                        .withPosition(4, 0),
                Mk4SwerveModuleHelper.GearRatio.L2,
                Constants.DriveConstants.BACK_LEFT_MODULE_DRIVE_MOTOR,
                Constants.DriveConstants.BACK_LEFT_MODULE_STEER_MOTOR,
                Constants.DriveConstants.BACK_LEFT_MODULE_STEER_ENCODER,
                Constants.DriveConstants.BACK_LEFT_MODULE_STEER_OFFSET, "BL"
        );
    
        SwerveModule m_backRightModule = Mk4SwerveModuleHelper.createFalcon500(
                tab.getLayout("Back Right Module", BuiltInLayouts.kList)
                        .withSize(2, 4)
                        .withPosition(6, 0),
                Mk4SwerveModuleHelper.GearRatio.L2,
                Constants.DriveConstants.BACK_RIGHT_MODULE_DRIVE_MOTOR,
                Constants.DriveConstants.BACK_RIGHT_MODULE_STEER_MOTOR,
                Constants.DriveConstants.BACK_RIGHT_MODULE_STEER_ENCODER,
                Constants.DriveConstants.BACK_RIGHT_MODULE_STEER_OFFSET, "BR"
        );

        Gyroscope gyro = GyroscopeHelper.createnavXMXP();

        realModules.add(m_frontLeftModule);
        realModules.add(m_frontRightModule);
        realModules.add(m_backLeftModule);
        realModules.add(m_backRightModule);
       

        
        


        return new SwerveDrivetrainModel(realModules, gyro);
        






    }
    public static SwerveSubsystem createSwerveSubsystem(SwerveDrivetrainModel dt) {
        return new SwerveSubsystem(dt);        
    }



    private static void passConstants() {
        SwerveConstants.MAX_FWD_REV_SPEED_MPS = Constants.DriveConstants.MAX_FWD_REV_SPEED_MPS;
        SwerveConstants.MAX_VOLTAGE = Constants.DriveConstants.MAX_VOLTAGE;
        SwerveConstants.DFLT_START_POSE = Constants.DriveConstants.DFLT_START_POSE;

        SwerveConstants.THETACONTROLLERkP = Constants.AutoConstants.THETACONTROLLERkP;
        SwerveConstants.THETACONTROLLERkI = Constants.AutoConstants.THETACONTROLLERkI;
        SwerveConstants.THETACONTROLLERkD = Constants.AutoConstants.THETACONTROLLERkD;

        SwerveConstants.THETACONTROLLERCONSTRAINTS = Constants.AutoConstants.THETACONTROLLERCONSTRAINTS;

        SwerveConstants.TRACKWIDTH_METERS = Constants.DriveConstants.TRACKWIDTH_METERS;
        SwerveConstants.TRACKLENGTH_METERS = Constants.DriveConstants.WHEELBASE_METERS;
        SwerveConstants.MASS_kg = Constants.DriveConstants.MASS_kg;
        SwerveConstants.MOI_KGM2 = Constants.DriveConstants.MOI_KGM2;
        SwerveConstants.KINEMATICS = Constants.DriveConstants.KINEMATICS;



        SwerveConstants.TRAJECTORYXkP = Constants.AutoConstants.TRAJECTORYXkP;
        SwerveConstants.TRAJECTORYXkI = Constants.AutoConstants.TRAJECTORYXkI;
        SwerveConstants.TRAJECTORYXkD = Constants.AutoConstants.TRAJECTORYXkD;


        SwerveConstants.TRAJECTORYYkP = Constants.AutoConstants.TRAJECTORYYkP;
        SwerveConstants.TRAJECTORYYkI = Constants.AutoConstants.TRAJECTORYYkI; 
        SwerveConstants.TRAJECTORYYkD = Constants.AutoConstants.TRAJECTORYYkD;
        
    }




    
    
}
