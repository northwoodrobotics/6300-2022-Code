// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.ExternalLib.RangerLib.LookupTable;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final class VisionConstants{
        public static final double TargetHeight = Units.inchesToMeters(16);
        public static final double blindlightHeight = Units.inchesToMeters(7.5);
        public static final double blindlightAngle = 1;

    }
    public static final class DriveConstants{
        public static final class AimConstants{ 
            public static final double AimP =13 ;           
            public static final double AimI =0;
            public static final double AimD =0.5;
        }

        
        /**
     * The left-to-right distance between the drivetrain wheels
     *
     * Should be measured from center to center.
     */
        public static final double TRACKWIDTH_METERS = 0.71; // FIXME Measure and set trackwidth
        /**
         * The front-to-back distance between the drivetrain wheels.
         *
         * Should be measured from center to center.
         */
        public static final double WHEELBASE_METERS = 0.71; // FIXME Measure and set wheelbase

        public static final SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(
                new Translation2d(TRACKWIDTH_METERS / 2.0, WHEELBASE_METERS / 2.0),
                new Translation2d(TRACKWIDTH_METERS / 2.0, -WHEELBASE_METERS / 2.0),
                new Translation2d(-TRACKWIDTH_METERS / 2.0, WHEELBASE_METERS / 2.0),
                new Translation2d(-TRACKWIDTH_METERS / 2.0, -WHEELBASE_METERS / 2.0)
        );

        public static final double WHEEL_DIAMETER_METERS = 0.10033; // .10033 = ~4 inches
        public static final double WHEEL_CIRCUMFERENCE_METERS = WHEEL_DIAMETER_METERS * Math.PI;

        public static final int PIGEON_ID = 0; // FIXME Set Pigeon ID

        public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 5; // FIXME Set front left module drive motor ID
        public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 4; // FIXME Set front left module steer motor ID
        public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 6; // FIXME Set front left steer encoder ID
        public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(129.3); // FIXME Measure and set front left steer offset

        public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 8; // FIXME Set front right drive motor ID
        public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 7; // FIXME Set front right steer motor ID
        public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 9; // FIXME Set front right steer encoder ID
        public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(251.98); // FIXME Measure and set front right steer offset

        public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 11; // FIXME Set back left drive motor ID
        public static final int BACK_LEFT_MODULE_STEER_MOTOR = 10; // FIXME Set back left steer motor ID
        public static final int BACK_LEFT_MODULE_STEER_ENCODER = 12; // FIXME Set back left steer encoder ID
        public static final double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(256.8); // FIXME Measure and set back left steer offset

        public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 15; // FIXME Set back right drive motor ID
        public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 14; // FIXME Set back right steer motor ID
        public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 16; // FIXME Set back right steer encoder ID
        public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(209.5); // FIXME Measure and set back right steer offset        

        // Drivetrain Performance Mechanical limits
        static public final double MAX_FWD_REV_SPEED_MPS = Units.feetToMeters(12);
        static public final double MAX_STRAFE_SPEED_MPS = Units.feetToMeters(12);
        static public final double MAX_ROTATE_SPEED_RAD_PER_SEC = Units.degreesToRadians(180);
        static public final double MAX_TRANSLATE_ACCEL_MPS2 = MAX_FWD_REV_SPEED_MPS/0.25; //0-full time of 0.25 second
        static public final double MAX_ROTATE_ACCEL_RAD_PER_SEC_2 = MAX_ROTATE_SPEED_RAD_PER_SEC/0.25; //0-full time of 0.25 second
        public static final double MAX_VOLTAGE = 12.0; // Maximum Voltage sent to the drive motors

        /////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // SENSOR CONSTANTS
        // Sensor-related constants - pulled from datasheets for the sensors and gearboxes
        static public final int ENC_PULSE_PER_REV = 2048; // TalonFX integrated sensor
        static public final int WHEEL_ENC_COUNTS_PER_WHEEL_REV = ENC_PULSE_PER_REV;  //Assume 1-1 gearing for now
        static public final int STEER_ENC_COUNTS_PER_MODULE_REV = 4096; // CANCoder
        static public final double WHEEL_ENC_WHEEL_REVS_PER_COUNT  = 1.0/((double)(WHEEL_ENC_COUNTS_PER_WHEEL_REV));
        static public final double steer_ENC_MODULE_REVS_PER_COUNT = 1.0/((double)(STEER_ENC_COUNTS_PER_MODULE_REV));
        static public final Pose2d DFLT_START_POSE = new Pose2d(Units.feetToMeters(24.0), Units.feetToMeters(10.0), Rotation2d.fromDegrees(0));
        static public final double ROBOT_MASS_kg = Units.lbsToKilograms(30);
        static public final double ROBOT_MOI_KGM2 = 1.0/12.0 * ROBOT_MASS_kg * Math.pow((WHEELBASE_METERS*1.1),2) * 2;
        public static final double MASS_kg = Units.lbsToKilograms(30);
        public static final double MOI_KGM2 = 1.0/12.0 * MASS_kg * Math.pow((TRACKWIDTH_METERS*1.1),2) * 2;
        // degrees per second
        public static final double Min_Rotation_Deg = 25;

       


    }
    public final static class AutoConstants{
        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularSpeedRadiansPerSecond =Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

        public static final double TRAJECTORYXkP = 0.94791;
        public static final double TRAJECTORYXkI = 0;
        public static final double TRAJECTORYXkD = 0;

        public static final double TRAJECTORYYkP = 0.94791;
        public static final double TRAJECTORYYkI = 0;
        public static final double TRAJECTORYYkD = 0;
        public static final double DriveKS = 1.1152;
        public static final double DriveKV = 0.62013;
        public static final double DriveKA = 0.12412;



        public static final double THETACONTROLLERkP = 1;
        public static final double THETACONTROLLERkI = 0.01;
        public static final double THETACONTROLLERkD = 0;



        

        // Constraint for the motion profilied robot angle controller
        public static final TrapezoidProfile.Constraints THETACONTROLLERCONSTRAINTS =
            new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    }
    public final static class ClimberConstants{
        public static final double RevEncoder_CountsPer_Rev = 8192;

        public static final double SpoolDiameter = Units.inchesToMeters(1);

        public static final double MidRungSetpoint = Units.inchesToMeters(0);




        
    }

    public final static class IntakeConstants{
        public static final int IntakeMotorID = 0;
        public static final int IntakeSolenoidID = 0; 
        public static final double IntakeMotorP = 1;
        public static final double IntakeMotorI = 0;
        public static final double IntakeMotorD = 0;

    }
    public final static class FeederConstants{
        public static final int FeederMotorID = 0;
        public static final int FeederStage1Sensor = 0;
        public static final int FeederStage2Sensor= 0;
        public static final int IntakeSensor = 0;




    }

    public final static class ShooterConstants{
        public static final int ShooterID = 20;
        public static final int ShooterFollowerID = 21;
        public static final int HoodID = 30;
        public static final int HoodServoID =0;
        public static final int HoodServo2ID = 2;
        public static final int HoodEncoderID = 0;

        public static final double HoodP = 0.01;
        public static final double HoodI = 0;
        public static final double HoodD = 0;
        public static final double HoodIZone = 0;
        public static final double HoodFF = 0;
        public static final double HoodMaxOutput = 1;
        public static final double HoodMinOutput = -1;

        public static final double ShooterGearRatio = 1.4;
        public static final double ShooterP = 0.01;
        public static final double ShooterI = 0.0;
        public static final double ShooterD = 0.0;
        //public static final double ShooerFF = .05;
        public static final double Shooter_AllowableError = 200;
        public static final double ShooterCurrentLimit = 10.0;


        public static final double ShooterPositonSensorCoffiecient = 1.0/2048 * 2;
        public static final double ShooterVelocitySensorCoffiecient = ShooterPositonSensorCoffiecient* (1000/100)*60;
        public static final double ShooterFF = 0.05;
        public static final double StaticFriction = 0.54;

        public static final LookupTable ShooterVelocityTable = new LookupTable();
        static{
            // tune
            ShooterVelocityTable.put(0,0);
        }

        public static final double HoodMinAngle = 16;
        public static final double HoodMaxAngle = -30;

        public static final double HoodOffset = 0.0;

        public static final LookupTable HoodPositionTable = new LookupTable();
        static {
            //tune this for as many as you want
            HoodPositionTable.put(0,0);
        }




    }

    



}
