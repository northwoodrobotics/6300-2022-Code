package frc.robot.commands.SubsystemCommands.TurretCommands;

import javax.xml.crypto.dsig.TransformException;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.Vision;
import frc.swervelib.SwerveSubsystem;

public class TurretTrack extends CommandBase{
    private final SwerveSubsystem m_SwerveSubsystem;
    private final TurretSubsystem m_turret;
    private final ShooterSubsystem m_shooter; 
    private final Vision m_vision; 
    private Pose2d RobotPose;
    private Transform2d vectorToGoal; 
    public TurretTrack(SwerveSubsystem swerve, TurretSubsystem turret, ShooterSubsystem shooter, Vision vision){
        m_SwerveSubsystem = swerve;
        m_turret = turret; 
        m_shooter = shooter; 
        m_vision = vision; 
    }
    @Override
    public void initialize() {

    }
    @Override
    public void execute() {
        RobotPose = m_SwerveSubsystem.dt.getPose(); 
        vectorToGoal = VisionConstants.GoalPose.minus(RobotPose);
        double turretToGoal = vectorToGoal.getRotation().getDegrees()-RobotPose.getRotation().getDegrees(); 
        if (!m_vision.hasTarget()){
            m_turret.setTurretAngle(turretToGoal);
        }else if(m_vision.hasTarget()){
            m_turret.setTurretAngle(turretToGoal- m_vision.getTargetAngleX());
        }




    }      
    
    
}
