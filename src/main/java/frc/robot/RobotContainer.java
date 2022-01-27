// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.VisionSubsystem;
import frc.swervelib.SwerveDrivetrainModel;
import frc.swervelib.SwerveSubsystem;
import frc.robot.commands.DriveCommands.CalibrateGyro;
import frc.robot.commands.DriveCommands.TeleopDriveCommand;
//import frc.robot.commands.TurnToTarget;
import frc.robot.commands.AutoRoutines.DriveAndTurn;
import frc.robot.commands.AutoRoutines.JustSquare;
import frc.robot.commands.ActionCommands.*;
import frc.robot.commands.AutoRoutines.DemoSquare;
import frc.robot.commands.AutoRoutines.RealSquare;
import frc.robot.commands.DriveCommands.ZeroGyro;
import frc.robot.commands.SimCommands.SimAuton;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
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
 
  public static SwerveDrivetrainModel dt = DrivetrainSubsystem.createSwerveModel();
  public static SwerveSubsystem m_swerveSubsystem = DrivetrainSubsystem.createSwerveSubsystem(dt);
  
 
  private static final SendableChooser<Command> autoChooser = new SendableChooser<>();

  //public static VisionSubsystem blindlight = new VisionSubsystem(m_swerveSubsystem);

  public static Vision blindlight = new Vision();









  

  public static final SpectrumXboxController driveController = new SpectrumXboxController(0, .1, .1);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

   
    m_swerveSubsystem.setDefaultCommand(new TeleopDriveCommand(  m_swerveSubsystem, 
    () -> modifyAxis(driveController.leftStick.getY()) * Constants.DriveConstants.MAX_FWD_REV_SPEED_MPS,
          () -> modifyAxis(-driveController.leftStick.getX()) * Constants.DriveConstants.MAX_STRAFE_SPEED_MPS,
          () -> modifyAxis(driveController.rightStick.getX()) *Constants.DriveConstants.MAX_ROTATE_SPEED_RAD_PER_SEC));
    
      
    // Configure the button bindings
    configureButtonBindings();

    ShowInputs();
    showBlindlight();
    
    SmartDashboard.putData("Auto Chooser", autoChooser);
    autoChooser.setDefaultOption("DriveAndTurn", new DriveAndTurn());
    autoChooser.addOption("DemoSquare", new DemoSquare());
    autoChooser.addOption("RealSquare", new RealSquare());
    autoChooser.addOption("No Rotation Square", new JustSquare());
    autoChooser.addOption("SimTrajectory", new SimAuton());


    
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

    driveController.xButton.whenHeld(
      new RotateToTarget(m_swerveSubsystem, 
      () -> driveController.leftStick.getY() * Constants.DriveConstants.MAX_FWD_REV_SPEED_MPS,
            () -> driveController.leftStick.getX() * Constants.DriveConstants.MAX_STRAFE_SPEED_MPS,
            () -> driveController.rightStick.getX() *Constants.DriveConstants.MAX_ROTATE_SPEED_RAD_PER_SEC

      ));
    driveController.aButton.whenHeld(new ZeroGyro(m_swerveSubsystem));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();

    // An ExampleCommand will run in autonomous
  
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
    master.addNumber("Gyro Yaw", () -> dt.getYaw().getDegrees());
    master.addNumber("Gyro Angle", () -> dt.getAngle());
    master.addNumber("Gyro Fused", () -> dt.getFused().getDegrees());
    
    
  }

  public void showBlindlight(){
    master.addBoolean("RobotHasTarget", ()->blindlight.hasTarget());
  }


}
