package frc.ExternalLib.JackInTheBotLib.kinematics;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.ExternalLib.JackInTheBotLib.math.Rotation2;
import frc.ExternalLib.JackInTheBotLib.math.Vector2;
import frc.ExternalLib.NorthwoodLib.MathWrappers.*;
import edu.wpi.first.math.geometry.Rotation2d;


/**
 * Helper class for swerve drive odometry.
 * <p>
 * Odometry allows a robot to track what it's position on the field is using the encoders on it's swerve modules.
 */
public class SwerveOdometry {
    private final SwerveKinematics kinematics;
    private NWPose2d pose;

    public SwerveOdometry(SwerveKinematics kinematics) {
        this(kinematics, NWPose2d.ZERO);
    }

    public SwerveOdometry(SwerveKinematics kinematics, NWPose2d initialPose) {
        this.kinematics = kinematics;
        this.pose = initialPose;
    }

    /**
     * Resets the robot's pose.
     *
     * @param pose The robot's new pose.
     */
    public void resetPose(NWPose2d pose) {
        this.pose = pose;
    }

    /**
     * Resets the robot's pose.
     *
     * @param position The new position of the robot.
     * @param rotation The new rotation of the robot.
     */
    public void resetPose(NWTranslation2d position, NWRotation2d rotation) {
        resetPose(new NWPose2d(position, rotation));
    }

    /**
     * Resets the robot's position.
     *
     * @param position The robot's new position.
     */
    public void resetPosition(NWTranslation2d position) {
        resetPose(position, (NWRotation2d) getPose().getRotation());
    }

    /**
     * Resets the robot's rotation.
     * <p>
     * This should be called when the gyroscope's angle is reset.
     *
     * @param rotation The robot's new rotation.
     */
    public void resetRotation(NWRotation2d rotation) {
        resetPose((NWTranslation2d) getPose().getTranslation(), rotation);
    }

    /**
     * Gets the position of the robot.
     *
     * @return The pose of the robot.
     */
    public NWPose2d getPose() {
        return pose;
    }

    /**
     * Updates the robot's position using forward kinematics and integration of the pose over time.
     *
     * @param gyroAngle        The angle from the gyroscope.
     * @param dt               The change in time.
     * @param moduleVelocities The velocities of the swerve modules. The modules must be in the same order that
     *                         {@link SwerveKinematics} was given when it was instantiated.
     * @return The new pose of the robot.
     */
    public Pose2d update(NWRotation2d gyroAngle, double dt, NWTranslation2d... moduleVelocities) {
        ChassisVelocity velocity = kinematics.toChassisVelocity(moduleVelocities);

        // Calculate the field-oriented translational velocity of the robot
        NWTranslation2d fieldOrientedVelocity = (NWTranslation2d) velocity.getTranslationalVelocity().rotateBy(gyroAngle);

        // Integrate using dt to determine our new position
        NWTranslation2d newPosition = (NWTranslation2d) pose.getTranslation()
                .plus(fieldOrientedVelocity.times(dt));

        pose = new NWPose2d(newPosition, gyroAngle);

        return pose;
    }
}
