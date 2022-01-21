// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.swervelib.SwerveDrivetrainModel;
import frc.swervelib.SwerveSubsystem;
import frc.robot.commands.TeleopDriveCommand;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.ExternalLib.SpectrumLib.controllers.SpectrumXboxController;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private static SwerveDrivetrainModel dt;
  private static SwerveSubsystem m_swerveSubsystem;



  private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);

  public static final SpectrumXboxController driveController = new SpectrumXboxController(0);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    dt = DrivetrainSubsystem.createSwerveModel();
    m_swerveSubsystem = DrivetrainSubsystem.createSwerveSubsystem(dt);
    m_swerveSubsystem.setDefaultCommand(new TeleopDriveCommand(  m_swerveSubsystem, 
    () -> -modifyAxis(driveController.leftStick.getY() * Constants.DriveConstants.MAX_FWD_REV_SPEED_MPS),
          () -> -modifyAxis(driveController.leftStick.getX() * Constants.DriveConstants.MAX_STRAFE_SPEED_MPS),
          () -> -modifyAxis(driveController.rightStick.getX() *Constants.DriveConstants.MAX_ROTATE_SPEED_RAD_PER_SEC)));
    
      
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {}

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_autoCommand;
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
}
