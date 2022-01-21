package frc.ExternalLib.JackInTheBotLib.control;

import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;
import java.util.TreeMap;

import frc.ExternalLib.JackInTheBotLib.math.Rotation2;
import frc.ExternalLib.JackInTheBotLib.math.Vector2;
import frc.ExternalLib.NorthwoodLib.MathWrappers.NWRotation2d;
import frc.ExternalLib.NorthwoodLib.MathWrappers.NWTranslation2d;

public final class SimplePathBuilder {
    private List<PathSegment> segmentList = new ArrayList<>();
    private TreeMap<Double, NWRotation2d> rotationMap = new TreeMap<>();

    private NWTranslation2d lastPosition;
    private double length = 0.0;

    public SimplePathBuilder(NWTranslation2d initialPosition,NWRotation2d initialRotation) {
        this.lastPosition = initialPosition;

        rotationMap.put(0.0, initialRotation);
    }

    private void addSegment(PathSegment segment) {
        segmentList.add(segment);
        length += segment.getLength();
        lastPosition = segment.getEnd().getPosition();
    }

    private void addSegment(PathSegment segment, NWRotation2d rotation) {
        addSegment(segment);
        rotationMap.put(length, rotation);
    }

    public Path build() {
        return new Path(segmentList.toArray(new PathSegment[0]), rotationMap);
    }

    public SimplePathBuilder arcTo(NWTranslation2d position, NWTranslation2d center) {
        addSegment(new ArcSegment(lastPosition, position, center));
        return this;
    }

    public SimplePathBuilder arcTo(NWTranslation2d position, NWTranslation2d center, NWRotation2d rotation) {
        addSegment(new ArcSegment(lastPosition, position, center), rotation);
        return this;
    }

    public SimplePathBuilder lineTo(NWTranslation2d position) {
        addSegment(new LineSegment(lastPosition, position));
        return this;
    }

    public SimplePathBuilder lineTo(NWTranslation2d position, NWRotation2d rotation) {
        addSegment(new LineSegment(lastPosition, position), rotation);
        return this;
    }

    public static final class ArcSegment extends PathSegment {
        private final NWTranslation2d center;
        private final NWTranslation2d deltaStart;
        private final NWTranslation2d deltaEnd;
        private final boolean clockwise;

        public ArcSegment(NWTranslation2d start, NWTranslation2d end, NWTranslation2d center) {
            this.center = center;
            this.deltaStart = (NWTranslation2d) start.minus(center);
            this.deltaEnd = (NWTranslation2d)end.minus(center);

            clockwise = deltaStart.cross(deltaEnd) <= 0.0;
        }

        @Override
        public State calculate(double distance) {
            double percentage = distance / getLength();

            double angle = NWTranslation2d.getAngleBetween(deltaStart, deltaEnd).getRadians() *
                    (clockwise ? -1.0 : 1.0) * percentage;
            return new State(
                    (NWTranslation2d) center.plus(deltaStart.rotateBy(NWRotation2d.fromRadians(angle))),
                    // TODO: Use cross product instead of just adding 90deg when calculating heading
                    ((NWTranslation2d) deltaStart.rotateBy(NWRotation2d.fromRadians(angle + (clockwise ? -1.0 : 1.0) * 0.5 * Math.PI))).getAngle(),
                    1.0 / deltaStart.length
            );
        }

        @Override
        public double getLength() {
            return deltaStart.length *NWTranslation2d.getAngleBetween(deltaStart, deltaEnd).getRadians();
        }
    }

    public static final class LineSegment extends PathSegment {
        private final NWTranslation2d start;
        private final NWTranslation2d delta;

        private LineSegment(NWTranslation2d start, NWTranslation2d end) {
            this.start = start;
            this.delta = (NWTranslation2d) end.minus(start);
        }

        @Override
        public State calculate(double distance) {
            return new State(
                    (NWTranslation2d) start.plus(delta.times(distance / getLength())),
                    delta.getAngle(),
                    0.0
            );
        }

        @Override
        public double getLength() {
            return delta.length;
        }
    }
}
