package frc.robot.commands.DriveCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.swervelib.SwerveSubsystem;
import java.util.function.DoubleSupplier;
public class AutonTurnToTarget extends CommandBase{
    private final SwerveSubsystem m_SwerveSubsystem;
    PIDController rotationController;
    public AutonTurnToTarget(SwerveSubsystem subsystem){
        this.m_SwerveSubsystem = subsystem;
        rotationController = new PIDController(Constants.DriveConstants.AimConstants.AimP, Constants.DriveConstants.AimConstants.AimI, Constants.DriveConstants.AimConstants.AimD);
        rotationController.setTolerance(0.5);

        
    }
    
    @Override
    public void initialize() {
//        rotationController.reset(-RobotContainer.limelight.getTargetAngleX());
    }
    @Override
    public void execute() {
        double XTranslation = 0;
        double YTranslation = 0;
        double Rotation = 0;
        double output = rotationController.calculate(RobotContainer.blindlight.getTargetAngleX(), 0);

        if(!rotationController.atSetpoint() && XTranslation == 0 && YTranslation == 0) {
            if(output < 0) output = Math.min(-Constants.DriveConstants.Min_Rotation_Deg, output);
            else output = Math.max(Constants.DriveConstants.Min_Rotation_Deg, output);
        }
        if(!RobotContainer.blindlight.hasTarget()){
            output = Rotation * Constants.DriveConstants.MAX_ROTATE_SPEED_RAD_PER_SEC;
        }
        m_SwerveSubsystem.dt.setModuleStates(Constants.DriveConstants.KINEMATICS.toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(
        XTranslation, 
        -YTranslation, 
         output, 
        m_SwerveSubsystem.dt.getGyroscopeRotation())
    ));



    }
    @Override
    public void end(boolean interrupted) {
        m_SwerveSubsystem.dt.setModuleStates(new ChassisSpeeds(0, 0, 0));
//        
    }

    @Override
    public boolean isFinished() {
        return false;
    }




    


    




    
    
}
