package frc.ExternalLib.JackInTheBotLib.control;

import java.util.Map;

import frc.ExternalLib.JackInTheBotLib.math.Rotation2;
import frc.ExternalLib.JackInTheBotLib.math.Vector2;
import frc.ExternalLib.JackInTheBotLib.util.InterpolatingDouble;
import frc.ExternalLib.JackInTheBotLib.util.InterpolatingTreeMap;
import frc.ExternalLib.NorthwoodLib.MathWrappers.NWRotation2d;
import frc.ExternalLib.NorthwoodLib.MathWrappers.NWTranslation2d;

public class Path {
    private final PathSegment[] segments;
    private final InterpolatingTreeMap<InterpolatingDouble, NWRotation2d> rotationMap = new InterpolatingTreeMap<>();
    private final double[] distancesFromStart;

    private final double length;

    public Path(PathSegment[] segments, Map<Double, NWRotation2d> rotationMap) {
        this.segments = segments;

        for (Map.Entry<Double, NWRotation2d> rotationEntry : rotationMap.entrySet()) {
            this.rotationMap.put(new InterpolatingDouble(rotationEntry.getKey()), rotationEntry.getValue());
        }

        distancesFromStart = new double[segments.length];
        double length = 0.0;
        for (int i = 0; i < segments.length; i++) {
            distancesFromStart[i] = length;
            length += segments[i].getLength();
        }
        this.length = length;
    }

    private double getDistanceToSegmentStart(int segment) {
        return distancesFromStart[segment];
    }

    private double getDistanceToSegmentEnd(int segment) {
        return distancesFromStart[segment] + segments[segment].getLength();
    }

    private int getSegmentAtDistance(double distance) {
        int start = 0;
        int end = segments.length - 1;
        int mid = start + (end - start) / 2;

        while (start <= end) {
            mid = (start + end) / 2;

            if (distance > getDistanceToSegmentEnd(mid)) {
                start = mid + 1;
            } else if (distance < getDistanceToSegmentStart(mid)) {
                end = mid - 1;
            } else {
                break;
            }
        }

        return mid;
    }

    public State calculate(double distance) {
        int currentSegment = getSegmentAtDistance(distance);
        PathSegment segment = segments[currentSegment];
        double segmentDistance = distance - getDistanceToSegmentStart(currentSegment);

        PathSegment.State state = segment.calculate(segmentDistance);

        return new Path.State(
                distance,
                state.getPosition(),
                state.getHeading(),
                rotationMap.getInterpolated(new InterpolatingDouble(distance)),
                state.getCurvature()
        );
    }

    public double getLength() {
        return length;
    }

    public PathSegment[] getSegments() {
        return segments;
    }

    public InterpolatingTreeMap<InterpolatingDouble, NWRotation2d> getRotationMap() {
        return rotationMap;
    }

    public static class State {
        private final double distance;
        private final NWTranslation2d position;
        private final NWRotation2d heading;
        private final NWRotation2d rotation;
        private final double curvature;

        public State(double distance, NWTranslation2d position, NWRotation2d heading, NWRotation2d rotation, double curvature) {
            this.distance = distance;
            this.position = position;
            this.heading = heading;
            this.rotation = rotation;
            this.curvature = curvature;
        }

        public double getDistance() {
            return distance;
        }

        public NWTranslation2d getPosition() {
            return position;
        }

        public NWRotation2d getHeading() {
            return heading;
        }

        public NWRotation2d getRotation() {
            return rotation;
        }

        public double getCurvature() {
            return curvature;
        }
    }
}
