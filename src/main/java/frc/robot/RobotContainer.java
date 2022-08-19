// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.MusicMaker;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Vision.LEDMode;
import frc.swervelib.SwerveDrivetrainModel;
import frc.swervelib.SwerveSubsystem;
import frc.robot.commands.DriveCommands.CalibrateGyro;
import frc.robot.commands.DriveCommands.DriveAutoRotate;
import frc.robot.commands.DriveCommands.TeleopDriveCommand;
import frc.robot.commands.ActionCommands.*;
import frc.robot.commands.AutoRoutines.FourBall;
import frc.robot.commands.AutoRoutines.OneBall;
import frc.robot.commands.AutoRoutines.SystemsCheck;
import frc.robot.commands.AutoRoutines.TopSideTwoBall;
import frc.robot.commands.AutoRoutines.TwoBall;
import frc.robot.commands.AutoRoutines.SixPlusOne;
import frc.robot.commands.SubsystemCommands.FeederCommands.PurgeFeeder;
import frc.robot.commands.SubsystemCommands.FeederCommands.RunFeeder;
import frc.robot.commands.SubsystemCommands.IntakeCommands.IntakeDeploy;
import frc.robot.commands.SubsystemCommands.IntakeCommands.IntakeUp;
import frc.robot.commands.SubsystemCommands.ShooterCommands.FenderShot;

import frc.robot.commands.SubsystemCommands.ShooterCommands.LowShot;
import frc.robot.commands.SubsystemCommands.ShooterCommands.ShooterCommand;
import frc.robot.commands.SubsystemCommands.TurretCommands.TurretTrack;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.ExternalLib.SpectrumLib.controllers.SpectrumXboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  private static ShuffleboardTab master = Shuffleboard.getTab("master");
  // The robot's subsystems and commands are defined here...
 
  public static SwerveDrivetrainModel dt;
  public static SwerveSubsystem m_swerveSubsystem;
  public static FeederSubsystem feeder; 
  public static IntakeSubsystem intake; 
  public static ClimberSubsystem climber;
  public static TurretSubsystem turret;
  public static ShooterSubsystem shooter;
  
 
  private static final SendableChooser<Command> autoChooser = new SendableChooser<>();

  public static Vision blindlight;

  double hoodAngle;
  double shooterSpeed;












  

  public static final SpectrumXboxController driveController = new SpectrumXboxController(0, .1, .1);
  public static final SpectrumXboxController gunnerController = new SpectrumXboxController(1, 0.1, 0.1);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
      feeder = new FeederSubsystem();
      blindlight = new Vision();
      shooter = new ShooterSubsystem();
      climber = new ClimberSubsystem();
      turret = new TurretSubsystem();
  


      intake = new IntakeSubsystem();
  
      dt = DrivetrainSubsystem.createSwerveModel();
      m_swerveSubsystem = DrivetrainSubsystem.createSwerveSubsystem(dt);
      m_swerveSubsystem.setDefaultCommand(new TeleopDriveCommand(  m_swerveSubsystem, 
    () -> modifyAxis(driveController.leftStick.getY()) * Constants.DriveConstants.MAX_FWD_REV_SPEED_MPS,
          () -> modifyAxis(-driveController.leftStick.getX()) * Constants.DriveConstants.MAX_STRAFE_SPEED_MPS,
          () -> modifyAxis(driveController.rightStick.getX()) *Constants.DriveConstants.MAX_ROTATE_SPEED_RAD_PER_SEC));


           // Configure the button bindings

    configureButtonBindings();
    ShowInputs();
    showBlindlight();
    turret.setDefaultCommand(new AutoLead(m_swerveSubsystem, turret, shooter, blindlight, true));
    feeder.setDefaultCommand(new RunFeeder(feeder));

    intake.setDefaultCommand(new IntakeDeploy(intake, 1));

 
    
    
    autoChooser.setDefaultOption("TwoBall", new TwoBall(m_swerveSubsystem, shooter, blindlight, feeder, intake));
    autoChooser.addOption("FourBall", new FourBall(m_swerveSubsystem, shooter, blindlight, feeder, intake));
    autoChooser.addOption("SystemsCheck", new SystemsCheck(m_swerveSubsystem, shooter, blindlight, feeder, intake));
    autoChooser.addOption("TopSideTwoBall", new TopSideTwoBall(m_swerveSubsystem, shooter, blindlight, feeder, intake));
    autoChooser.addOption("OneBall", new OneBall(m_swerveSubsystem, shooter, blindlight, feeder, intake));
    autoChooser.addOption("WackyTwoBall", new SixPlusOne(m_swerveSubsystem, shooter, blindlight, feeder, intake));
   

   
    

    
   
      
   
    
    
      
   

    
  }
  

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    /*driveController.xButton.whileHeld(new TurnToTarget(m_swerveSubsystem, blindlight,
    () -> -modifyAxis(-driveController.leftStick.getY()) * Constants.DriveConstants.MAX_STRAFE_SPEED_MPS ,
    () -> -modifyAxis(driveController.leftStick.getX()) * Constants.DriveConstants.MAX_FWD_REV_SPEED_MPS
    ));*/
    driveController.startButton.whenPressed(()->
      m_swerveSubsystem.dt.zeroGyroscope()
    );
    driveController.rightTriggerButton.whenPressed(new PurgeFeeder(feeder));
    
    
       
    driveController.xButton.toggleWhenPressed(new IntakeUp(intake, 1));
     

     
     
      


    
    
   
   

  }
  


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();

    
  
 
  }

  
  
  private static double deadband(double value, double deadband) {
    if (Math.abs(value) > deadband) {
      if (value > 0.0) {
        return (value - deadband) / (1.0 - deadband);
      } else {
        return (value + deadband) / (1.0 - deadband);
      }
    } else {
      return 0.0;
    }
  }

  
  private static double modifyAxis(double value) {
    // Deadband
    value = deadband(value, 0.05);

    // Square the axis
    value = Math.copySign(value * value, value);

    return value;
  }
  public void ShowInputs(){
    master.addNumber("X Command", ()-> -modifyAxis(driveController.leftStick.getX()) * Constants.DriveConstants.MAX_STRAFE_SPEED_MPS);
    master.addNumber("Y Command", () -> -modifyAxis(driveController.leftStick.getY()) * Constants.DriveConstants.MAX_FWD_REV_SPEED_MPS);
    
    master.addNumber("X Input", ()-> -modifyAxis(driveController.leftStick.getX()));
    master.addNumber("Y Input", () -> -modifyAxis(driveController.leftStick.getY()));
    master.addNumber("GyroReading", () -> dt.getGyroscopeRotation().getDegrees());
    master.addBoolean("OnTarget", ()->blindlight.isOnTarget());
    
    master.addNumber("PoseX", ()-> m_swerveSubsystem.dt.getPose().getX());
    master.addNumber("PoseY", ()-> m_swerveSubsystem.dt.getPose().getY());
    master.addNumber("PoseRotation", ()-> m_swerveSubsystem.dt.getPose().getRotation().getDegrees());
    master.addNumber("Shooter Velocity", ()->shooter.shooterSpeed());
   

    
    
    
    
    
  }




  



  public void showBlindlight(){
    master.addBoolean("RobotHasTarget", ()->blindlight.hasTarget());
  }


}
