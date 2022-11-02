package frc.robot.commands.DriveCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.Vision;
import frc.swervelib.SwerveSubsystem;
import java.util.function.DoubleSupplier;
public class DriveAutoRotate extends CommandBase {
  // auto snap command, runing a PID loop based off of the limelight, auto corrects to the hub. 

    private ChassisSpeeds stop = new ChassisSpeeds(0, 0, 0);
 
    private final SwerveSubsystem m_SwerveSubsystem;
    
    private final DoubleSupplier m_translationXSupplier;
    private final DoubleSupplier m_translationYSupplier;
    private final Vision m_Limelight;
    
    private Transform2d vectorToGoal; 
    public DriveAutoRotate( SwerveSubsystem subsystem, DoubleSupplier translationXSupplier,
    DoubleSupplier translationYSupplier, Vision LimeLight) {
        addRequirements(subsystem);
        this.m_SwerveSubsystem = subsystem;
        this.m_translationXSupplier = translationXSupplier;
        this.m_translationYSupplier = translationYSupplier;
        this.m_Limelight = LimeLight;
       

       
    }

    @Override
    public void initialize() {
       

    }

    @Override
    public void execute() {

        double XTranslation = m_translationXSupplier.getAsDouble();
        double YTranslation = m_translationYSupplier.getAsDouble();
       
        double angleToGoal = vectorToGoal.getRotation().getDegrees()-m_Limelight.getTargetAngleX(); 
        vectorToGoal = VisionConstants.GoalPose.minus(m_SwerveSubsystem.dt.getPose());


       
      
        if (Math.abs(angleToGoal) > 1){
            m_SwerveSubsystem.dt.driveClean(XTranslation, YTranslation, angleToGoal);
        }else m_SwerveSubsystem.dt.driveClean(XTranslation, YTranslation, 0);
  

        
    }

    @Override
    public void end(boolean interrupted) {
        m_SwerveSubsystem.dt.setModuleStates(stop);
//        
    }

    @Override
    public boolean isFinished() {
        return Math.abs(RobotContainer.blindlight.getTargetAngleX()) <= 1;
    }
}