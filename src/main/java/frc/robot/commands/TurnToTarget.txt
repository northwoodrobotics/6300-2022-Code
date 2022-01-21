package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.ExternalLib.JackInTheBotLib.robot.drivers.Limelight;
import frc.ExternalLib.NorthwoodLib.MathWrappers.NWTranslation2d;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.swervelib.SwerveSubsystem;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.ExternalLib.JackInTheBotLib.control.PidConstants;
import frc.ExternalLib.JackInTheBotLib.control.PidController;
import java.util.function.DoubleSupplier;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;


public class TurnToTarget extends CommandBase{
    private static final PidConstants PID_CONSTANTS = new PidConstants(0.5, 2.0, 0.025);
    private static final double ROTATION_STATIC_CONSTANT = 0.3;
    private static  final double MAXIMUM_AVERAGE_VELOCITY = 2.0;

    private final SwerveSubsystem subsystem;
    private final VisionSubsystem vision;
    private final DoubleSupplier xAxis;
    private final DoubleSupplier yAxis;

    private PidController controller = new PidController(PID_CONSTANTS);
    private double lastTime = 0.0;




    
    public TurnToTarget(SwerveSubsystem drivetrain, VisionSubsystem visionSubsystem,
                                       DoubleSupplier xAxis, DoubleSupplier yAxis) {
        this.subsystem = drivetrain;
        this.vision= visionSubsystem;
        this.xAxis = xAxis;
        this.yAxis = yAxis;
        addRequirements(drivetrain);
        addRequirements(visionSubsystem);

        controller.setInputRange(0.0, 2.0 * Math.PI);
        controller.setContinuous(true);
        controller.setIntegralRange(Math.toRadians(10.0));
    }
    @Override
    public void initialize() {
        lastTime = Timer.getFPGATimestamp();
        vision.setCamMode(Limelight.CamMode.VISION);
        vision.setSnapshotEnabled(true);
        controller.reset();
    }
    @Override
    public void execute() {
        double time = Timer.getFPGATimestamp();
        double dt = time - lastTime;
        lastTime = time;

        Translation2d translationalVelocity = new Translation2d(xAxis.getAsDouble(), yAxis.getAsDouble());

        double rotationalVelocity = 0.0;
        if (vision.hasTarget()) {
            double currentAngle = subsystem.dt.getGyroscopeRotation().getRadians();
            double targetAngle = vision.getAngleToTarget().getAsDouble();
            controller.setSetpoint(targetAngle);
            rotationalVelocity = controller.calculate(currentAngle, dt);

            if (subsystem.dt.getAverageAbsoluteValueVelocity() < MAXIMUM_AVERAGE_VELOCITY) {
                rotationalVelocity += Math.copySign(
                        ROTATION_STATIC_CONSTANT / RobotController.getBatteryVoltage(),
                        rotationalVelocity
                );
            }
        }
        subsystem.dt.setModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(translationalVelocity.getX(), 
        translationalVelocity.getY(), 
        rotationalVelocity, 
        subsystem.dt.getGyroscopeRotation()));

        
    }
 

}
