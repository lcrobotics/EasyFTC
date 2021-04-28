package com.lcrobotics.easyftclib.pathfinding;

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
        return Math.abs(p1.getX() - p2.getX()) < buffer && Math.abs(p1.getY() - p2.getY()) < buffer;
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
        return Math.abs(a1 - a2) < buffer;
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

        final double dthetaAbsolute = Math.atan2(dyAbsolute, dxAbsolute);
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

    public static List<Translation2d> lineCircleIntersection(Translation2d circleCenter, double radius,
            Translation2d linePoint1, Translation2d linePoint2) {
        // This method was lifted from Team 11115 Gluten Free's code.

        double baX = linePoint2.getX() - linePoint1.getX();
        double baY = linePoint2.getY() - linePoint1.getY();
        double caX = circleCenter.getX() - linePoint1.getX();
        double caY = circleCenter.getY() - linePoint1.getY();

        double a = baX * baX + baY * baY;
        double bBy2 = baX * caX + baY * caY;
        double c = caX * caX + caY * caY - radius * radius;

        double pBy2 = bBy2 / a;
        double q = c / a;

        double disc = pBy2 * pBy2 - q;
        if (disc < 0) {
            return Collections.emptyList();
        }

        double tmpSqrt = Math.sqrt(disc);
        double abScalingFactor1 = -pBy2 + tmpSqrt;
        double abScalingFactor2 = -pBy2 - tmpSqrt;

        List<Translation2d> allPoints = null;

        Translation2d p1 = new Translation2d(linePoint1.getX() - baX * abScalingFactor1,
                linePoint1.getY() - baY * abScalingFactor1);
        if (disc == 0) {
            allPoints = Collections.singletonList(p1);
        }

        if (allPoints == null) {
            Translation2d p2 = new Translation2d(linePoint1.getX() - baX * abScalingFactor2,
                    linePoint1.getY() - baY * abScalingFactor2);
            allPoints = Arrays.asList(p1, p2);
        }

        double maxX = Math.max(linePoint1.getX(), linePoint2.getX());
        double maxY = Math.max(linePoint1.getY(), linePoint2.getY());
        double minX = Math.min(linePoint1.getX(), linePoint2.getX());
        double minY = Math.min(linePoint1.getY(), linePoint2.getY());

        // make sure points are on the line segment
        return allPoints.stream().filter(
                (p) ->
                        p.getX() <= maxX && p.getX() >= minX
                                && p.getY() <= maxY && p.getY() >= minY
        ).collect(Collectors.toList());
    }
}
