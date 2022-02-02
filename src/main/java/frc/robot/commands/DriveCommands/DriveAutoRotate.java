package frc.robot.commands.DriveCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.swervelib.SwerveSubsystem;
import java.util.function.DoubleSupplier;
public class DriveAutoRotate extends CommandBase {
    /**
     * Creates a new DriveTurnToTarget
     * */

    private ChassisSpeeds stop = new ChassisSpeeds(0, 0, 0);
 
    private final SwerveSubsystem m_SwerveSubsystem;
    
    private final DoubleSupplier m_translationXSupplier;
    private final DoubleSupplier m_translationYSupplier;
    private final DoubleSupplier m_rotationSupplier;
    PIDController rotationController;
    public DriveAutoRotate( SwerveSubsystem subsystem, DoubleSupplier translationXSupplier,
    DoubleSupplier translationYSupplier,
    DoubleSupplier rotationSupplier) {
        addRequirements(subsystem);
        this.m_SwerveSubsystem = subsystem;
        this.m_translationXSupplier = translationXSupplier;
        this.m_translationYSupplier = translationYSupplier;
        this.m_rotationSupplier = rotationSupplier;

        rotationController = new PIDController(Constants.DriveConstants.AimConstants.AimP, Constants.DriveConstants.AimConstants.AimI, Constants.DriveConstants.AimConstants.AimD);
        rotationController.setTolerance(0.5);
    }

    @Override
    public void initialize() {
//        rotationController.reset(-RobotContainer.limelight.getTargetAngleX());
    }

    @Override
    public void execute() {

        double XTranslation = m_translationXSupplier.getAsDouble();
        double YTranslation = m_translationYSupplier.getAsDouble();
        double Rotation = m_rotationSupplier.getAsDouble();


        SmartDashboard.putNumber("Targeting Error", rotationController.getPositionError());

//        if(Math.abs(rotationController.getPositionError()) <= Constants.DRIVE_TARGETING_I_ZONE){
//            rotationController.setI(Constants.DRIVE_TARGETING_CONTROLLER_I);
//        }else{
//            rotationController.setI(0);
//        }

        double output = rotationController.calculate(RobotContainer.blindlight.getTargetAngleX(), 0);
        SmartDashboard.putNumber("Targeting Output", output);
        if(!rotationController.atSetpoint() && XTranslation == 0 && YTranslation == 0) {
            if(output < 0) output = Math.min(-Constants.DriveConstants.Min_Rotation_Deg, output);
            else output = Math.max(Constants.DriveConstants.Min_Rotation_Deg, output);
        }
        if(!RobotContainer.blindlight.hasTarget()){
            output = Rotation * Units.radiansToDegrees(Constants.DriveConstants.MAX_ROTATE_SPEED_RAD_PER_SEC);
        }
//        System.out.println(output);
   m_SwerveSubsystem.dt.setModuleStates(Constants.DriveConstants.KINEMATICS.toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(
        XTranslation, 
        -YTranslation, 
         output, 
        m_SwerveSubsystem.dt.getGyroscopeRotation())
    ));

        
    }

    @Override
    public void end(boolean interrupted) {
        m_SwerveSubsystem.dt.setModuleStates(stop);
//        
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}