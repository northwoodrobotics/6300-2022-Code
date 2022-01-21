package frc.ExternalLib.JackInTheBotLib.math.spline;

import org.ejml.simple.SimpleMatrix;

import frc.ExternalLib.JackInTheBotLib.math.Rotation2;
import frc.ExternalLib.JackInTheBotLib.math.Vector2;
import frc.ExternalLib.NorthwoodLib.MathWrappers.NWRotation2d;
import frc.ExternalLib.NorthwoodLib.MathWrappers.NWTranslation2d;

class HermiteSplineHelper {
    /**
     * Creates the basis weight matrix for a cubic hermite spline.
     *
     * @param start        The position of the spline at t = 0.
     * @param startTangent The tangent of the spline at t = 0.
     * @param end          The position of the spline at t = 1.
     * @param endTangent   The tangent of the spline at t = 1.
     * @return The basis weights for the spline.
     */
    public static SimpleMatrix createBasisWeightMatrix(NWTranslation2d start, NWTranslation2d startTangent,
    NWTranslation2d end, NWTranslation2d endTangent) {
        // The basis weight matrix for hermite cubic splines is the following:
        // [x0  y0 ]
        // [x1  y1 ]
        // [dx0 dy0]
        // [dx1 dy1]
        return new SimpleMatrix(new double[][]{
                new double[]{start.getX(), start.getY()},
                new double[]{end.getX(), end.getY()},
                new double[]{startTangent.getX(), startTangent.getY()},
                new double[]{endTangent.getX(), endTangent.getX()}
        });
    }

    /**
     * Creates the basis weight matrix for a cubic hermite spline.
     *
     * @param start        The position of the spline at t = 0.
     * @param startHeading The heading of the spline at t = 0.
     * @param end          The position of the spline at t = 1.
     * @param endHeading   The heading of the spline at t = 1.
     * @return The basis weights for the spline.
     */
    public static SimpleMatrix createBasisWeightMatrix(NWTranslation2d start, NWRotation2d startHeading,
    NWTranslation2d end, NWRotation2d endHeading) {
        double scale = 2.0 * ((NWTranslation2d) end.minus(start)).getLength();

        return createBasisWeightMatrix(
                start, (NWTranslation2d) NWTranslation2d.fromAngle(startHeading).times(scale),
                end, (NWTranslation2d) NWTranslation2d.fromAngle(endHeading).times(scale)
        );
    }
}
