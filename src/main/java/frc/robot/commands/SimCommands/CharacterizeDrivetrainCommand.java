package frc.robot.commands.SimCommands;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.swervelib.SwerveSubsystem;


public class CharacterizeDrivetrainCommand  extends CommandBase{
    private final SwerveSubsystem subsystem;
    private final NetworkTableEntry autoSpeedEntry =
            NetworkTableInstance.getDefault().getEntry("/SmartDashboard/SysIdAutoSpeed");
    private final NetworkTableEntry telemetryEntry =
            NetworkTableInstance.getDefault().getEntry("/SmartDashboard/SysIdTelemetry");

    private List<Double> telemetryData = new ArrayList<>();

    private double priorAutospeed = 0.0;

    public CharacterizeDrivetrainCommand(SwerveSubsystem subsystem) {
        this.subsystem = subsystem;

        addRequirements(subsystem);
    }

    @Override
    public void initialize(){
        subsystem.dt.setKnownPose(new Pose2d());
        subsystem.dt.zeroGyroscope();
        NetworkTableInstance.getDefault().setUpdateRate(10.0e-3);


    }
    @Override
    public void execute(){
        double now = Timer.getFPGATimestamp();
        double position = subsystem.dt.getCurActPose().getTranslation().getX();
        double velocity = subsystem.dt.getAverageAbsoluteValueVelocity();
        double battery = RobotController.getBatteryVoltage();
        double motorVoltage = battery * Math.abs(priorAutospeed);
        double autospeed = autoSpeedEntry.getDouble(0.0);
        priorAutospeed = autospeed;

        subsystem.dt.setModuleStates(DriveConstants.KINEMATICS.toSwerveModuleStates(new ChassisSpeeds(autospeed, 0,0)));
        telemetryData.add(now);
        telemetryData.add(autospeed * RobotController.getInputVoltage());
        telemetryData.add(position);
        telemetryData.add(velocity);





    }
    @Override
    public void end(boolean interrupted) {
        StringBuilder b = new StringBuilder();
        for (int i = 0; i < telemetryData.size(); ++i) {
            if (i != 0)
                b.append(", ");
            b.append(telemetryData.get(i));
        }

        telemetryEntry.setString(b.toString());
        subsystem.dt.setModuleStates(DriveConstants.KINEMATICS.toSwerveModuleStates(new ChassisSpeeds(0, 0, 0)));




    }
}
