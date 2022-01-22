package frc.robot.commands;


import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.swervelib.SwerveSubsystem;
import frc.robot.Constants;
import java.util.function.DoubleSupplier;
import frc.ExternalLib.SpectrumLib.controllers.SpectrumXboxController;




public class TeleopDriveCommand extends CommandBase{
    private final SwerveSubsystem m_SwerveSubsystem;
    private final SpectrumXboxController drivecontroller;
    private final DoubleSupplier m_translationXSupplier;
    private final DoubleSupplier m_translationYSupplier;
    private final DoubleSupplier m_rotationSupplier;

    private double m_translationY;
    private double m_translationX;
    private double m_rotation;


    public TeleopDriveCommand( SwerveSubsystem subsystem, DoubleSupplier translationXSupplier,
    DoubleSupplier translationYSupplier,
    DoubleSupplier rotationSupplier){
        this.m_SwerveSubsystem = subsystem;
        this.m_translationXSupplier = translationXSupplier;
        this.m_translationYSupplier = translationYSupplier;
        this.m_rotationSupplier = rotationSupplier;

        this.drivecontroller = RobotContainer.driveController;
        addRequirements(subsystem);


    }


    @Override
    public void execute(){
      m_SwerveSubsystem.dt.setModuleStates(Constants.DriveConstants.KINEMATICS.toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(
        m_translationXSupplier.getAsDouble(), 
        -m_translationYSupplier.getAsDouble(), 
        m_rotationSupplier.getAsDouble(), 
        m_SwerveSubsystem.dt.getGyroscopeRotation())));
       
      
      
      
      /*m_SwerveSubsystem.dt.setModuleStates(Constants.DriveConstants.KINEMATICS.toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(
            m_translationX*Constants.DriveConstants.MAX_FWD_REV_SPEED_MPS, m_translationY*Constants.DriveConstants.MAX_STRAFE_SPEED_MPS, 
            m_rotation*Constants.DriveConstants.MAX_STRAFE_SPEED_MPS, m_SwerveSubsystem.dt.getGyroscopeRotation())));*/
    }

    @Override
    public void end(boolean interrupted) {
        m_SwerveSubsystem.dt.setModuleStates(
          Constants.DriveConstants.KINEMATICS.toSwerveModuleStates(  
            new ChassisSpeeds(0.0, 0.0, 0.0)
          )    
        );


    }
    
  private static double modifyAxis(double value) {
    // Square the axis
    value = Math.copySign(value * value, value);

    return value;
}






    
}
