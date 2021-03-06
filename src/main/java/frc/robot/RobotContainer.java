// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.swervedrivespecialties.swervelib.DriveController;

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
import frc.robot.subsystems.Vision;
//import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.Vision.LEDMode;
import frc.swervelib.SwerveDrivetrainModel;
import frc.swervelib.SwerveSubsystem;
import frc.robot.commands.DriveCommands.CalibrateGyro;
import frc.robot.commands.DriveCommands.DriveAutoRotate;
import frc.robot.commands.DriveCommands.TeleopDriveCommand;
//import frc.robot.commands.TurnToTarget;
//import frc.robot.commands.AutoRoutines.DriveAndTurn;
//import frc.robot.commands.AutoRoutines.JustSquare;
import frc.robot.commands.ActionCommands.*;
import frc.robot.commands.AutoRoutines.FourBall;
import frc.robot.commands.AutoRoutines.OneBall;
//import frc.robot.commands.AutoRoutines.OneBallRed;
import frc.robot.commands.AutoRoutines.SystemsCheck;
import frc.robot.commands.AutoRoutines.TopSideTwoBall;
import frc.robot.commands.AutoRoutines.TwoBall;
import frc.robot.commands.AutoRoutines.WackyTwoBall;
//import frc.robot.commands.AutoRoutines.TwoBallRed;
//import frc.robot.commands.AutoRoutines.DemoSquare;
//import frc.robot.commands.AutoRoutines.DriveAndGoLeft;
//import frc.robot.commands.AutoRoutines.RealSquare;
import frc.robot.commands.BlindLightCommands.LimelightSwitchLEDMode;
import frc.robot.commands.DriveCommands.ZeroGyro;
import frc.robot.commands.MusicLibary.PlaySelectedSong;
import frc.robot.commands.SimCommands.HoodUp;
//import frc.robot.commands.SimCommands.SimAuton;
import frc.robot.commands.SimCommands.TuneTables;
import frc.robot.commands.SubsystemCommands.PurgeFeeder;
import frc.robot.commands.SubsystemCommands.RunFeeder;
import frc.robot.commands.SubsystemCommands.FenderShot;
//import frc.robot.commands.SubsystemCommands.SetServoMax;
//import frc.robot.commands.SubsystemCommands.SetServoMid;
//import frc.robot.commands.SubsystemCommands.SetServoMin;
import frc.robot.commands.SubsystemCommands.HomeHood;
import frc.robot.commands.SubsystemCommands.IntakeCommand;
import frc.robot.commands.SubsystemCommands.LowShot;
import frc.robot.commands.SubsystemCommands.PurgeFeeder;
import frc.robot.commands.SubsystemCommands.ShooterCommand;
import frc.robot.commands.SubsystemCommands.WaitForBall;
import frc.robot.commands.SubsystemCommands.AutoCommands.AutoFeedCommand;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;
import frc.ExternalLib.SpectrumLib.controllers.SpectrumXboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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

 // public static XboxController yeet = new XboxController (1);
  //public static JoystickButton leftTrigger = new JoystickButton(yeet, XboxController.Button.kLeftBumper.value);
  

  public static MusicMaker musicMaker;
  public static Compressor compressor;

  private DrivetrainMode driveControlMode = DrivetrainMode.DRIVE;

  //private DriveAndTurn driveandTurn;

  public static ShooterSubsystem shooter;
  
 
  private static final SendableChooser<Command> autoChooser = new SendableChooser<>();
  //private static final SendableChooser<DrivetrainMode> DriveModeChooser = new SendableChooser<>();
  //private static final SendableChooser<Command> SongChooser = new SendableChooser<>();
  //private static final SendableChooser<Double> SpeedChooser = new SendableChooser<>();
  //public static VisionSubsystem blindlight = new VisionSubsystem(m_swerveSubsystem);

  public static Vision blindlight;
  private NetworkTableEntry m_shooterSpeed = master.add("ShooterSpeed", -7000).getEntry();
  private NetworkTableEntry m_hoodAngle = master.add("hoodAngle", 1).getEntry();
  double hoodAngle;
  double shooterSpeed;












  

  public static final SpectrumXboxController driveController = new SpectrumXboxController(0, .1, .1);
  public static final SpectrumXboxController DJController = new SpectrumXboxController(1, 0.1, 0.1);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
      feeder = new FeederSubsystem();
      blindlight = new Vision();
      shooter = new ShooterSubsystem();
      climber = new ClimberSubsystem();

      compressor = new Compressor(31, PneumaticsModuleType.REVPH);
    //  yeet = new XboxController(1);
    //shooter.setHoodTargetAngle(37);

      //yeet.getAButtonPressed()


      intake = new IntakeSubsystem();
      compressor.enabled();
      compressor.enableDigital();
    //switch(driveControlMode){
     // case DRIVE :
     // musicMaker = null;
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
    //driveandTurn = new DriveAndTurn(m_swerveSubsystem);
    //SmartDashboard.putNumber("Shooter RPM", ShooterSpeed);

    
    
    
    autoChooser.setDefaultOption("TwoBall", new TwoBall(m_swerveSubsystem, shooter, blindlight, feeder, intake));
    autoChooser.addOption("FourBall", new FourBall(m_swerveSubsystem, shooter, blindlight, feeder, intake));
    autoChooser.addOption("SystemsCheck", new SystemsCheck(m_swerveSubsystem, shooter, blindlight, feeder, intake));
    autoChooser.addOption("TopSideTwoBall", new TopSideTwoBall(m_swerveSubsystem, shooter, blindlight, feeder, intake));
    autoChooser.addOption("OneBall", new OneBall(m_swerveSubsystem, shooter, blindlight, feeder, intake));
    autoChooser.addOption("WackyTwoBall", new WackyTwoBall(m_swerveSubsystem, shooter, blindlight, feeder, intake));
    /*autoChooser.addOption("DemoSquare", new DemoSquare(m_swerveSubsystem));
    autoChooser.addOption("RealSquare", new RealSquare(m_swerveSubsystem));
    autoChooser.addOption("No Rotation Square", new JustSquare(m_swerveSubsystem));
    autoChooser.addOption("SimTrajectory", new SimAuton(m_s   werveSubsystem));
    autoChooser.addOption("Training", new DriveAndGoLeft(m_swerveSubsystem));*/

     //SmartDashboard.getNumber("Shooter RPM", 0);
      

    
   

      //break;
   // case MUSIC :
    // dt = null;
   //  m_swerveSubsystem = null;
   //  musicMaker = new MusicMaker();
    // getMusicBindings();


     
     
    // break;
    // }
  //   SmartDashboard.putData("DriveModeChooser", DriveModeChooser);
    // DriveModeChooser.setDefaultOption("DriveMode", DrivetrainMode.DRIVE);
   //  DriveModeChooser.addOption("MusicMode", DrivetrainMode.MUSIC);
  
     SmartDashboard.putData("Auto Chooser", autoChooser);
     //SmartDashboard.putData("Song", SongChooser);


   
    

    
   
      
   
    
    
      
   

    
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
    driveController.startButton.whenHeld(
      new CalibrateGyro(m_swerveSubsystem)
    );
    driveController.rightBumper.whileHeld(new ParallelCommandGroup(new LowShot(shooter, blindlight), new SequentialCommandGroup(new WaitCommand(1), new PurgeFeeder(feeder, 1))));
    driveController.rightTriggerButton.whileHeld(
      new ParallelCommandGroup( new FenderShot(shooter, blindlight), new SequentialCommandGroup(new WaitCommand(1), new PurgeFeeder(feeder, 1))));
    driveController.leftTriggerButton.whileHeld(
      new ParallelCommandGroup(new ShooterCommand(shooter, blindlight),new SequentialCommandGroup(new WaitCommand(1), new PurgeFeeder(feeder, 1)))
      );
    driveController.bButton.whenPressed(()-> blindlight.setLEDMode(LEDMode.LED_OFF));
    driveController.leftBumper.whileHeld(new ParallelDeadlineGroup(new RotateToTarget(m_swerveSubsystem, 
    () -> driveController.leftStick.getY() * Constants.DriveConstants.MAX_FWD_REV_SPEED_MPS,
          () -> driveController.leftStick.getX() * Constants.DriveConstants.MAX_STRAFE_SPEED_MPS,
          () -> driveController.rightStick.getX() *Constants.DriveConstants.MAX_ROTATE_SPEED_RAD_PER_SEC)), true);
    /*driveController.xButton.whileHeld(
      new RotateToTarget(m_swerveSubsystem,   
      () -> driveController.leftStick.getY() * Constants.DriveConstants.MAX_FWD_REV_SPEED_MPS,
            () -> driveController.leftStick.getX() * Constants.DriveConstants.MAX_STRAFE_SPEED_MPS,
            () -> driveController.rightStick.getX() *Constants.DriveConstants.MAX_ROTATE_SPEED_RAD_PER_SEC

      ));*/
      //driveController.xButton.toggleWhenPressed(new IntakeCommand(intake, feeder, 0.35), true);

      /*leftTrigger.whenPressed(
        new ShooterCommand(shooter, blindlight)
      );*/
      driveController.xButton.toggleWhenPressed(new IntakeMasterCommand(feeder, intake)/*new SortBall(feeder)*/);
        //new IntakeMasterCommand(feeder, intake), true);
    // driveController.rightBumper.whileHeld(new PurgeFeeder(feeder, 0.45), true);
      //driveController.yButton.whileHeld(new PurgeFeeder(feeder, -45));
      //driveController.Dpad.Up.whenPressed(()-> shooter.setHoodTargetAngle((shooter.getHoodTargetAngle().orElse(ShooterConstants.HoodMaxAngle)+ 0.5)));
      //driveController.Dpad.Down.whenPressed(()-> shooter.setHoodTargetAngle((shooter.getHoodTargetAngle().orElse(ShooterConstants.HoodMaxAngle)- 0.5)));
      driveController.aButton.whileHeld(()->feeder.runFeeder(0.45));
      driveController.aButton.whenReleased(()-> feeder.runFeeder(0));
      //DJController.leftBumper.whenReleased(new PurgeFeeder(feeder, 0));
      driveController.yButton.whileHeld(()-> feeder.runFeeder(0.-45));
      driveController.yButton.whenReleased(()-> feeder.runFeeder(0));
      

    /*DJController.aButton.toggleWhenPressed(
      new PlaySelectedSong(shooter), true
    );*/
    DJController.leftBumper.whileHeld(()-> climber.ExtendClimb() );
    DJController.leftBumper.whenReleased(()-> climber.HoldClimb() );
    DJController.rightBumper.whileHeld(()-> climber.Climb() );
    DJController.rightBumper.whenReleased(()-> climber.HoldClimb() );
    
   
   

  }
  /*private void getMusicBindings(){
    driveController.aButton.whenHeld(
      new gasgasgas(musicMaker)
    );
    driveController.bButton.whenHeld(
      new pokerface(musicMaker)
    );
    driveController.xButton.whenHeld(
      new stayinalive(musicMaker)
    );
    driveController.yButton.whenHeld(
      new rickroll(musicMaker)
    );
  }*/



  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();

    // An ExampleCommand will run in autonomous
  
 
  }

  public static void UpdateTelemetry(){
    dt.updateTelemetry();
    //dt.Updateodometry();
    //dt.update(false, RobotController.getBatteryVoltage());
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
    //master.addNumber("Gyro Yaw", () -> dt.getYaw().getDegrees());
    //master.addNumber("Gyro Angle", () -> dt.getAngle());
    //master.addNumber("Gyro Fused", () -> dt.getFused().getDegrees());
    master.addNumber("PoseX", ()-> m_swerveSubsystem.dt.getPose().getX());
    master.addNumber("PoseY", ()-> m_swerveSubsystem.dt.getPose().getY());
    master.addNumber("PoseRotation", ()-> m_swerveSubsystem.dt.getPose().getRotation().getDegrees());
    master.addNumber("Shooter Velocity", ()->shooter.shooterSpeed());
    //master.addNumber("OdometryX", ()-> m_swerveSubsystem.dt.getPose().getX());
    //master.addNumber("OdometryY", ()-> m_swerveSubsystem.dt.getPose().getY());
    //master.addNumber("OdometryRotation", ()-> m_swerveSubsystem.dt.getPose().getRotation().getDegrees());z


    
    
    
    
    
  }



  public enum DrivetrainMode{
    DRIVE,
    MUSIC
  }

  public void setDriveMode(){
    driveControlMode = DrivetrainMode.DRIVE;
  }
  public void setMusicMode(){
    driveControlMode = DrivetrainMode.MUSIC;
  }
  



  public void showBlindlight(){
    master.addBoolean("RobotHasTarget", ()->blindlight.hasTarget());
  }


}
