package frc.ExternalLib.JackInTheBotLib.math.spline;

import org.ejml.simple.SimpleMatrix;

import frc.ExternalLib.JackInTheBotLib.math.Rotation2;
import frc.ExternalLib.JackInTheBotLib.math.Vector2;
import frc.ExternalLib.NorthwoodLib.MathWrappers.NWRotation2d;
import frc.ExternalLib.NorthwoodLib.MathWrappers.NWTranslation2d;

public class CubicHermiteSpline extends Spline {
    private static final SimpleMatrix BASIS_MATRIX = new SimpleMatrix(new double[][]{
            new double[]{1, 0, 0, 0},
            new double[]{0, 0, 1, 0},
            new double[]{-3, 3, -2, -1},
            new double[]{2, -2, 1, 1},
    });
    private static final SimpleMatrix INVERSE_BASIS_MATRIX = BASIS_MATRIX.invert();

    public CubicHermiteSpline(NWTranslation2d start, NWTranslation2d startTangent,
    NWTranslation2d end, NWTranslation2d endTangent) {
        this(HermiteSplineHelper.createBasisWeightMatrix(start, startTangent, end, endTangent));
    }

    public CubicHermiteSpline(NWTranslation2d start, NWRotation2d startHeading,
                              NWTranslation2d end, NWRotation2d endHeading) {
        this(HermiteSplineHelper.createBasisWeightMatrix(start, startHeading, end, endHeading));
    }

    private CubicHermiteSpline(SimpleMatrix basisWeightMatrix) {
        super(BASIS_MATRIX, basisWeightMatrix);
    }

    /**
     * Converts a cubic spline to a cubic spline with a hermite representation.
     *
     * @param spline The spline to convert.
     * @return The hermite representation of the spline.
     */
    public static CubicHermiteSpline convert(Spline spline) {
        if (spline.getDegree() != 3) {
            throw new IllegalArgumentException("Spline must be cubic.");
        }

        // B1 * W1 = B2 * W2
        // W1 = B1^-1 * B2 * W2
        return new CubicHermiteSpline(INVERSE_BASIS_MATRIX.mult(spline.getBasisMatrix()).mult(spline.getBasisWeightMatrix()));
    }
}
