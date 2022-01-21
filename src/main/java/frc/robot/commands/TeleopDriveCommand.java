package frc.robot.commands;


import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.swervelib.SwerveSubsystem;
import frc.robot.Constants;




public class TeleopDriveCommand extends CommandBase{
    private final SwerveSubsystem m_SwerveSubsystem;
    private final XboxController drivecontroller;

    private double m_translationY;
    private double m_translationX;
    private double m_rotation;


    public TeleopDriveCommand( SwerveSubsystem subsystem){
        this.m_SwerveSubsystem = subsystem;

        this.drivecontroller = RobotContainer.driveController;


        m_translationX = modifyAxis(-drivecontroller.getLeftY());
        m_translationY = modifyAxis(-drivecontroller.getLeftX());
        m_rotation = modifyAxis(-drivecontroller.getRightX());

        addRequirements(subsystem);


    }


    @Override
    public void execute(){
        m_SwerveSubsystem.dt.setModuleStates(Constants.DriveConstants.KINEMATICS.toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(
            m_translationX*Constants.DriveConstants.MAX_FWD_REV_SPEED_MPS, m_translationY*Constants.DriveConstants.MAX_STRAFE_SPEED_MPS, 
            m_rotation*Constants.DriveConstants.MAX_ROTATE_SPEED_RAD_PER_SEC, m_SwerveSubsystem.dt.getGyroscopeRotation())));
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
