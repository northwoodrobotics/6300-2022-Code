package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.ExternalLib.JackInTheBotLib.math.MathUtils;
import frc.ExternalLib.JackInTheBotLib.math.Vector2;
import frc.ExternalLib.JackInTheBotLib.robot.drivers.Limelight;
import frc.robot.RobotContainer;
import frc.swervelib.SwerveSubsystem;

import java.util.OptionalDouble;

public class VisionSubsystem implements Subsystem {
    private static final double TARGET_HEIGHT = 98.25; // Change to height of 2022 goal

    private static final double LIMELIGHT_HEIGHT = 22.0; //change to actual robot Blindlight Height




    private static final double LIMELIGHT_MOUNTING_ANGLE = Math.toRadians(29.0); // change to blindlight angle on 2022 robot

    private static final double TARGET_ALLOWABLE_ERROR = Math.toRadians(2.5);

    private static final Limelight LIMELIGHT = new Limelight("Front");
   

    private final SwerveSubsystem drivetrain;

    private final NetworkTableEntry distanceToTargetEntry;
    private final NetworkTableEntry dXOuterEntry;
    private final NetworkTableEntry dYOuterEntry;
  

    private boolean hasTarget;
    private boolean isInnerTargetVisible;
    private OptionalDouble distanceToTarget = OptionalDouble.empty();
    private OptionalDouble angleToTarget = OptionalDouble.empty();

    public VisionSubsystem(SwerveSubsystem drivetrainSubsystem) {
        drivetrain = drivetrainSubsystem;
        ShuffleboardTab tab = Shuffleboard.getTab("Vision");
        distanceToTargetEntry = tab.add("distance to target", 0.0)
                .withPosition(0, 0)
                .withSize(1, 1)
                .getEntry();
        dXOuterEntry = tab.add("dXOuter", 0.0)
                .withPosition(1, 0)
                .withSize(1, 1)
                .getEntry();
        dYOuterEntry = tab.add("dYOuter", 0.0)
                .withPosition(2, 0)
                .withSize(1, 1)
                .getEntry();

        tab.addNumber("target angle", () -> Math.toDegrees(getAngleToTarget().orElse(Double.NaN)))
                .withPosition(4, 0)
                .withSize(1, 1);
        tab.addBoolean("Is on target", this::isOnTarget)
                .withPosition(5, 0)
                .withSize(1, 1);
        tab.addNumber("Horizontal Target Error", () -> {
                    double gyroAngle = drivetrain.dt.getGyroscopeRotation().getRadians();
                    return getDistanceToTarget().orElse(0.0) *
                            (Math.sin(gyroAngle - getAngleToTarget().orElse(0.0)) / Math.sin(Math.PI / 2.0 - gyroAngle));
                })
                .withPosition(6, 0)
                .withSize(1, 1);

      
    }

    public void setCamMode(Limelight.CamMode mode) {
        LIMELIGHT.setCamMode(mode);
    }

    public OptionalDouble getDistanceToTarget() {
        return distanceToTarget;
    }

    public OptionalDouble getAngleToTarget() {
        return angleToTarget;
    }

    public OptionalDouble getHorizontalError() {
        OptionalDouble distanceToTargetOpt = getDistanceToTarget();
        OptionalDouble angleToTargetOpt = getAngleToTarget();

        if (distanceToTargetOpt.isEmpty() || angleToTargetOpt.isEmpty()) {
            return OptionalDouble.empty();
        }

        double gyroAngle = drivetrain.dt.getGyroscopeRotation().getRadians();
        return OptionalDouble.of(distanceToTargetOpt.getAsDouble() *
                (Math.sin(gyroAngle - angleToTargetOpt.getAsDouble()) / Math.sin(Math.PI / 2.0 - gyroAngle)));
    }

    public boolean hasTarget() {
        return hasTarget;
    }

    public boolean isInnerTargetVisible() {
        return isInnerTargetVisible;
    }


    @Override
    public void periodic() {
        // Shooter limelight
        // Determine whether the Limelight has a target or not
        hasTarget = LIMELIGHT.hasTarget();
        if (hasTarget) {
            // Calculate the distance to the outer target
            Translation2d targetPosition = LIMELIGHT.getTargetPosition();
            double theta = LIMELIGHT_MOUNTING_ANGLE + targetPosition.getY();
            double distanceToOuterTarget = (TARGET_HEIGHT - LIMELIGHT_HEIGHT) / Math.tan(theta);

            // Get the field oriented angle for the outer target, with latency compensation
            double angleToOuter = drivetrain.dt.getGyroscopeRotation().getRadians() - targetPosition.getX();
            double dYOuter = distanceToOuterTarget * Math.sin(angleToOuter);
            double dXOuter = distanceToOuterTarget * Math.cos(angleToOuter);
            dXOuterEntry.setDouble(dXOuter);
            dYOuterEntry.setDouble(dYOuter);

            // Calculate the distance to the inner target

            // Add DISTANCE_FROM_INNER_TO_APEX to dXInner here because we want if we did it when we defined dXInner
            // distanceToInnerTarget would be incorrect, and we only need this extra bit to determine if we can see
            // the inner target
           

            // Check whether we can see the inner target
            
            
                distanceToTarget = OptionalDouble.of(distanceToOuterTarget);
            
            angleToTarget = OptionalDouble.of(angleToOuter);
        } else {
            distanceToTarget = OptionalDouble.empty();
            angleToTarget = OptionalDouble.empty();
            
        }

        // Intake limelight
     

        // Update shuffleboard
        distanceToTargetEntry.setDouble(distanceToTarget.orElse(-1.0));
        
    }

    public boolean isOnTarget() {
        OptionalDouble targetAngle = getAngleToTarget();
        if (targetAngle.isEmpty()) {
            return false;
        }

        double delta = targetAngle.getAsDouble() - drivetrain.dt.getGyroscopeRotation().getRadians();
        if (delta > Math.PI) {
            delta = 2.0 * Math.PI - delta;
        }

        return MathUtils.epsilonEquals(
                delta,
                0,
                TARGET_ALLOWABLE_ERROR
        );
    }

    public void setLedMode(Limelight.LedMode mode) {
        LIMELIGHT.setLedMode(mode);
    }

    public void setSnapshotEnabled(boolean isEnabled) {
        LIMELIGHT.setSnapshotsEnabled(isEnabled);
    }
}