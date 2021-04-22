package com.lcrobotics.easyftclib.pathfinding;

import android.util.Range;

import com.lcrobotics.easyftclib.tools.geometry.Pose2d;
import com.lcrobotics.easyftclib.tools.geometry.Translation2d;

import java.util.Arrays;
import java.util.Collections;
import java.util.List;
import java.util.stream.Collectors;

public final class PathfindingUtil {

    private PathfindingUtil() {
    }

    /**
     * Wraps the able so it is always in the range [-180, 180].
     *
     * @param angle Angle to be wrapped, in radians.
     * @return The wrapped angle, in radians.
     */
    public static double angleWrap(double angle) {
        if (angle > 0) {
            return ((angle + Math.PI) % (Math.PI * 2)) - Math.PI;
        } else {
            return ((angle - Math.PI) % (Math.PI * 2)) + Math.PI;
        }
    }

    /**
     * Calculates if a point is further along a line then another point. Useful for determining the best intersection
     * point in pure pursuit. Both points are assumed to lie on the line.
     *
     * @param linePoint1 First point of the line.
     * @param linePoint2 Second point of the line.
     * @param point1     Point to be compared.
     * @param point2     Point that point1 is compared too.
     * @return Whether point1 is ahead of point2 on the given line.
     */
    public static boolean isInFront(Translation2d linePoint1, Translation2d linePoint2,
            Translation2d point1, Translation2d point2) {

        if (linePoint1.getX() < linePoint2.getX() && point1.getX() < point2.getX()) {
            return false;
        }

        return !(linePoint1.getY() < linePoint2.getY()) || !(point1.getY() < point2.getY());
    }

    /**
     * Calculates whether two points are equal given a margin of error.
     *
     * @param p1     Point 1.
     * @param p2     Point 2.
     * @param buffer Margin of error.
     * @return Whether they are equal given the margin of error.
     */
    public static boolean positionEqualsWithBuffer(Translation2d p1, Translation2d p2, double buffer) {
        return p1.getDistance(p2) <= buffer;
    }

    /**
     * Calculates whether two angles are within a margin of error from each other.
     *
     * @param a1     Angle 1 (in radians).
     * @param a2     Angle 2 (in radians).
     * @param buffer Margin of error.
     * @return Whether the two angles are equal given the margin of error.
     */
    public static boolean rotationEqualsWithBuffer(double a1, double a2, double buffer) {
        return Math.abs(a1 - a2) <= buffer;
    }

    /**
     * @param current  The robot's current position.
     * @param target   Target position.
     * @param turnOnly Whether the robot should only turn.
     * @return An array of motor powers, containing, in this order, the strafe power, forward power, and turn power
     */
    public static double[] moveToPosition(Pose2d current, Pose2d target, boolean turnOnly) {

        if (turnOnly) {
            return new double[]{0, 0, angleWrap(target.getHeading() + current.getHeading()) / Math.PI};
        }

        final double dxAbsolute = target.getX() - current.getX();
        final double dyAbsolute = target.getY() - current.getY();

        final double dthetaAbsolute = Math.atan2(dxAbsolute, dyAbsolute);
        final double distance = Math.hypot(dxAbsolute, dyAbsolute);

        final double dthetaRelative = angleWrap(dthetaAbsolute + current.getHeading());

        final double dxRelative = distance * Math.cos(dthetaRelative);
        final double dyRelative = distance * Math.sin(dthetaRelative);

        final double manhattanDistance = Math.abs(dxRelative) + Math.abs(dyRelative);
        final double powerX = dxRelative / manhattanDistance;
        final double powerY = dyRelative / manhattanDistance;
        final double powerTurn = angleWrap(current.getHeading() + target.getHeading()) / Math.PI;

        return new double[]{powerX, powerY, powerTurn};
    }

    /**
     * This method finds points where a line intersects with a circle.
     *
     * @param circleCenter Center of the circle.
     * @param radius       Radius of the circle.
     * @param linePoint1   One of the line's end points.
     * @param linePoint2   The other end point of the line.
     * @return A list containing all points where the line and circle intersect.
     * @see
     * <a href=https://mathworld.wolfram.com/Circle-LineIntersection.html>https://mathworld.wolfram.com/Circle-LineIntersection.html</a>
     * for explanation
     */
    public static List<Translation2d> lineCircleIntersection(Translation2d circleCenter, double radius,
            Translation2d linePoint1, Translation2d linePoint2) {

        linePoint1 = linePoint1.minus(circleCenter);
        linePoint2 = linePoint2.minus(circleCenter);

        final double dx = linePoint2.getX() - linePoint1.getX();
        final double dy = linePoint2.getY() - linePoint1.getY();
        final double dr = Math.hypot(dx, dy);

        final double det = linePoint1.getX() * linePoint2.getY() - linePoint2.getX() * linePoint1.getY();

        final double discriminant = radius * radius * dr * dr - det * det;

        List<Translation2d> allPoints = null;
        // no intersections
        if (discriminant < 0) {
            return Collections.emptyList();
        }

        final double tempSqrt = Math.sqrt(discriminant);

        final double scaleX = tempSqrt * Math.signum(dy) * dx;
        final double scaleY = tempSqrt * Math.abs(dy);

        Translation2d p1 = new Translation2d(
                (det * dy + scaleX) / (dr * dr),
                (-det * dx + scaleY) / (dr * dr)
        );
        // tangent
        if (discriminant == 0) {
            allPoints = Collections.singletonList(p1);
        }
        // two intersections
        if (allPoints == null) {
            Translation2d p2 = new Translation2d(
                    (det * dy - scaleX) / (dr * dr),
                    (-det * dx - scaleY) / (dr * dr)
            );
            allPoints = Arrays.asList(p1, p2);
        }

        final double maxX = Math.max(linePoint1.getX(), linePoint2.getX());
        final double maxY = Math.max(linePoint1.getY(), linePoint2.getY());
        final double minX = Math.min(linePoint1.getX(), linePoint2.getX());
        final double minY = Math.min(linePoint1.getY(), linePoint2.getY());

        Range<Double> xInterval = new Range<>(minX, maxX);
        Range<Double> yInterval = new Range<>(minY, maxY);
        // only grab points on the line segment
        return allPoints
                .stream()
                .filter(p -> xInterval.contains(p.getX()) && yInterval.contains(p.getY()))
                .collect(Collectors.toList());
    }
}
