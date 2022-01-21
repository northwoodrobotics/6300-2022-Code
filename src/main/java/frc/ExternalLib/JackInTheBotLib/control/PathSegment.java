package frc.ExternalLib.JackInTheBotLib.control;

import frc.ExternalLib.JackInTheBotLib.math.Rotation2;
import frc.ExternalLib.JackInTheBotLib.math.Vector2;
import frc.ExternalLib.NorthwoodLib.MathWrappers.NWRotation2d;
import frc.ExternalLib.NorthwoodLib.MathWrappers.NWTranslation2d;

public abstract class PathSegment {
    public State getStart() {
        return calculate(0.0);
    }

    public State getEnd() {
        return calculate(getLength());
    }

    public abstract State calculate(double distance);

    public abstract double getLength();

    public static class State {
        private final NWTranslation2d position;
        private final NWRotation2d heading;
        private final double curvature;

        public State(NWTranslation2d position, NWRotation2d heading, double curvature) {
            this.position = position;
            this.heading = heading;
            this.curvature = curvature;
        }

        public NWTranslation2d getPosition() {
            return position;
        }

        public NWRotation2d getHeading() {
            return heading;
        }

        public double getCurvature() {
            return curvature;
        }
    }
}
