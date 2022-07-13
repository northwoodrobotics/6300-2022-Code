package frc.robot.commands.ActionCommands;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.ExternalLib.PoofLib.util.InterpolatingDouble;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.Vision;
import frc.swervelib.SwerveSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
public class AutoLead extends CommandBase{
    private final ShooterSubsystem m_shooter;
    private final TurretSubsystem m_turret;
    private final SwerveSubsystem m_drive;
    private final Vision m_blindlight;
    private final boolean m_updatePose;
    private final Timer m_timer = new Timer();
    private Pose2d RobotPose;
    private Transform2d vectorToGoal; 
    private Transform2d vectorToPredicted;
    public AutoLead(SwerveSubsystem drive, TurretSubsystem turret, ShooterSubsystem shooter, Vision blindlight, boolean updatePose){
        this.m_drive = drive;
        this.m_turret = turret;
        this.m_shooter = shooter;
        this.m_blindlight = blindlight; 
        this.m_updatePose = updatePose; 

    }
    @Override
    public void initialize(){

        
        
    }
    @Override
    public void execute(){
        ChassisSpeeds FieldVelocity = m_drive.dt.getFieldRelativeSpeeds();
        ChassisSpeeds FieldAccel = m_drive.dt.getFieldReltaiveAcceleration(); 
        ChassisSpeeds FieldJerk = m_drive.dt.getFieldRelativeJerk(); 
        vectorToGoal = Constants.VisionConstants.GoalPose.minus(RobotPose);

        double PredictedGoalX = VisionConstants.GoalPose.getX()-(FieldVelocity.vxMetersPerSecond*ShooterConstants.ShotTime.getInterpolated(new InterpolatingDouble(vectorToGoal.getTranslation().getDistance(RobotPose.getTranslation()))).value)+ (1/2*(FieldAccel.vxMetersPerSecond+(FieldJerk.vxMetersPerSecond*ShooterConstants.ShotTime.getInterpolated(new InterpolatingDouble(vectorToGoal.getTranslation().getDistance(RobotPose.getTranslation()))).value)))*Math.pow(ShooterConstants.ShotTime.getInterpolated(new InterpolatingDouble(vectorToGoal.getTranslation().getDistance(RobotPose.getTranslation()))).value, 2);
        double PredictedGoalY = VisionConstants.GoalPose.getY()-(FieldVelocity.vyMetersPerSecond*ShooterConstants.ShotTime.getInterpolated(new InterpolatingDouble(vectorToGoal.getTranslation().getDistance(RobotPose.getTranslation()))).value)+ (1/2*(FieldAccel.vyMetersPerSecond+(FieldJerk.vyMetersPerSecond*ShooterConstants.ShotTime.getInterpolated(new InterpolatingDouble(vectorToGoal.getTranslation().getDistance(RobotPose.getTranslation()))).value)))*Math.pow(ShooterConstants.ShotTime.getInterpolated(new InterpolatingDouble(vectorToGoal.getTranslation().getDistance(RobotPose.getTranslation()))).value, 2);

        Pose2d PredictedGoal = new Pose2d(PredictedGoalX, PredictedGoalY, Rotation2d.fromDegrees(0)); 

        RobotPose = m_drive.dt.getPose(); 
        vectorToGoal = Constants.VisionConstants.GoalPose.minus(RobotPose);
        vectorToPredicted = PredictedGoal.minus(RobotPose);
        double turretToGoal = vectorToGoal.getRotation().getDegrees()-RobotPose.getRotation().getDegrees(); 
        if (!m_blindlight.hasTarget()){
            m_turret.setTurretAngle(turretToGoal);
        }else if(m_blindlight.hasTarget()){
            m_turret.setTurretAngle(turretToGoal- m_blindlight.getTargetAngleX());

        }
        m_shooter.RunShooter(ShooterConstants.ShooterVelocityTable.getInterpolated(new InterpolatingDouble(vectorToPredicted.getTranslation().getDistance(RobotPose.getTranslation()))).value);
        m_shooter.setHoodTargetAngle((ShooterConstants.HoodPositionTable.getInterpolated(new InterpolatingDouble(vectorToPredicted.getTranslation().getDistance(RobotPose.getTranslation()))).value));



    }
}
