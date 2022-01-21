package frc.ExternalLib.JackInTheBotLib.control;

import frc.ExternalLib.JackInTheBotLib.math.Vector2;
import frc.ExternalLib.JackInTheBotLib.math.spline.Spline;
import frc.ExternalLib.NorthwoodLib.MathWrappers.NWTranslation2d;

public final class SplinePathSegment extends PathSegment {
    private static final double LENGTH_SAMPLE_STEP = 1.0e-4;

    private final Spline spline;

    private transient double length = Double.NaN;

    public SplinePathSegment(Spline spline) {
        this.spline = spline;
    }

    @Override
    public State calculate(double distance) {
        double t = distance / getLength();

        return new State(
                spline.getPoint(t),
                spline.getHeading(t),
                spline.getCurvature(t)
        );
    }

    @Override
    public double getLength() {
        if (!Double.isFinite(length)) {
            length = 0.0;
            NWTranslation2d p0 = spline.getPoint(0.0);
            for (double t = LENGTH_SAMPLE_STEP; t <= 1.0; t += LENGTH_SAMPLE_STEP) {
                NWTranslation2d p1 = spline.getPoint(t);
                length += ((NWTranslation2d) p1.minus(p0)).getLength();

                p0 = p1;
            }
        }

        return length;
    }

    public Spline getSpline() {
        return spline;
    }
}
