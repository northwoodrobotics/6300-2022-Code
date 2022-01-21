package frc.ExternalLib.JackInTheBotLib.kinematics;

import org.ejml.simple.SimpleMatrix;

import frc.ExternalLib.JackInTheBotLib.math.Vector2;
import frc.ExternalLib.NorthwoodLib.MathWrappers.NWTranslation2d;

import java.util.Arrays;

/**
 * Helper class that converts a chassis velocity (translational and rotation velocities) into individual swerve module
 * velocities.
 * <p>
 * Inverse kinematics (converting from a desired chassis velocity to individual module velocities) uses the relative
 * locations of the modules w.r.t. the center of the drive base to determine how each module should move in order to
 * achieve a desired chassis velocity.
 * <p>
 * Forward kinematics (converting a set of module velocities into an overall chassis velocity) performs the exact
 * opposite of what inverse kinematics does.
 * <p>
 * For an in-depth explanation of the underlying mathematics that this class performs, see section 13.4.3 of
 * <a href="https://file.tavsys.net/control/state-space-guide.pdf">Controls Engineering in the FIRST Robotics Competition</a>
 */
public class SwerveKinematics {
    private final NWTranslation2d[] moduleOffsets;

    private final SimpleMatrix inverseKinematics;
    private final SimpleMatrix forwardKinematics;

    public SwerveKinematics(NWTranslation2d... moduleOffsets) {
        if (moduleOffsets.length < 1) {
            throw new IllegalArgumentException("Must have at least 1 module");
        }

        this.moduleOffsets = Arrays.copyOf(moduleOffsets, moduleOffsets.length);

        inverseKinematics = new SimpleMatrix((int) (moduleOffsets.length * 2), 3);
        for (int i = 0; i < moduleOffsets.length; i++) {
            inverseKinematics.setRow(i * 2 + 0, 0, 1.0, 0.0, -moduleOffsets[i].getX());
            inverseKinematics.setRow(i * 2 + 1, 0, 0.0, 1.0, moduleOffsets[i].getX());
        }
        forwardKinematics = inverseKinematics.pseudoInverse();
    }

    /**
     * Performs inverse kinematics to convert a desired chassis velocity into a set of swerve module velocities.
     *
     * @param velocity The desired overall robot velocity.
     * @return An array containing the module velocities required to reach the desired robot velocity. These velocities
     * may be outside the acceptable range of the modules. Use the
     * {@link #normalizeModuleVelocities(Vector2[], double) normalizeModuleVelocities} method to resolve this issue.
     */
    public NWTranslation2d[] toModuleVelocities(ChassisVelocity velocity) {
        SimpleMatrix chassisVelocityVector = new SimpleMatrix(3, 1);
        chassisVelocityVector.setColumn(0, 0,
                velocity.getTranslationalVelocity().getX(),
                velocity.getTranslationalVelocity().getY(),
                velocity.getAngularVelocity());

        SimpleMatrix moduleVelocitiesMatrix = inverseKinematics.mult(chassisVelocityVector);
        NWTranslation2d[] moduleVelocities = new NWTranslation2d[moduleOffsets.length];

        for (int i = 0; i < moduleOffsets.length; i++) {
            moduleVelocities[i] = new NWTranslation2d(
                    moduleVelocitiesMatrix.get(i * 2 + 0),
                    moduleVelocitiesMatrix.get(i * 2 + 1)
            );
        }

        return moduleVelocities;
    }

    /**
     * Performs forward kinematics to convert a set of swerve module velocities into a chassis velocity.
     *
     * @param moduleVelocities The velocities of the modules w.r.t the robot.
     * @return The chassis velocity that would result from the module velocities.
     */
    public ChassisVelocity toChassisVelocity(NWTranslation2d... moduleVelocities) {
        if (moduleVelocities.length != moduleOffsets.length) {
            throw new IllegalArgumentException("Amount of module velocities given does not match the amount of modules specified in the constructor");
        }

        SimpleMatrix moduleVelocitiesMatrix = new SimpleMatrix(moduleOffsets.length * 2, 1);
        for (int i = 0; i < moduleOffsets.length; i++) {
            moduleVelocitiesMatrix.setColumn(0, i * 2,
                    moduleVelocities[i].getX(),
                    moduleVelocities[i].getY()
            );
        }

        SimpleMatrix chassisVelocityVector = forwardKinematics.mult(moduleVelocitiesMatrix);
        return new ChassisVelocity(
                new NWTranslation2d(
                        chassisVelocityVector.get(0),
                        chassisVelocityVector.get(1)
                ),
                chassisVelocityVector.get(2)
        );
    }

    /**
     * Normalizes the module velocities using some maximum velocity.
     * <p>
     * Sometimes, the required velocity from one or more modules may be above the modules maximum attainable velocity.
     * To fix this issue, all the module velocities can be "normalized" so that all module velocities are below the
     * maximum attainable velocity while still keeping the ratio of velocities between the modules.
     *
     * @param moduleVelocities An array containing the module velocities. This array will be mutated to contain the
     *                         normalized velocities.
     * @param maximumVelocity  The absolute maximum velocity that a module can reach.
     */
    public static void normalizeModuleVelocities(NWTranslation2d[] moduleVelocities, double maximumVelocity) {
        double realMaxVelocity = Arrays.stream(moduleVelocities).mapToDouble(m -> m.length).max().orElseThrow();
        if (realMaxVelocity > maximumVelocity) {
            for (int i = 0; i < moduleVelocities.length; i++) {
                moduleVelocities[i] = (NWTranslation2d) moduleVelocities[i].times(maximumVelocity / realMaxVelocity);
            }
        }
    }
}
