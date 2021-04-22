package com.lcrobotics.easyftclib.pathfinding.waypoints;

import com.lcrobotics.easyftclib.pathfinding.actions.InterruptAction;
import com.lcrobotics.easyftclib.pathfinding.types.WaypointType;
import com.lcrobotics.easyftclib.tools.geometry.Pose2d;
import com.lcrobotics.easyftclib.tools.geometry.Rotation2d;
import com.lcrobotics.easyftclib.tools.geometry.Translation2d;

public class EndWaypoint extends InterruptWaypoint {
    private boolean isFinished = false;

    /**
     * Default constructor for an EndWaypoint.
     */
    public EndWaypoint() {
        super.setAction(generateEndAction());
    }

    /**
     * Constructs an EndWaypoint with the provided values.
     *
     * @param translation    The (x, y) translation of this waypoint.
     * @param rotation       The rotation (preferred angle) of this waypoint.
     * @param movementSpeed  The speed in which the robot moves at while traversing this waypoint.
     * @param turnSpeed      The speed in which the robot turns at while traversing this waypoint.
     * @param followRadius   The distance in which the robot traverses this waypoint.
     * @param positionBuffer The expected level of error, the robot will consider itself at the waypoint when it is within the buffer. The buffer must be > 0.
     * @param rotationBuffer The expected level of error (in radians), the robot will consider itself at the waypoint when it is within the buffer. The buffer must be > 0.
     */
    public EndWaypoint(Translation2d translation, Rotation2d rotation, double movementSpeed, double turnSpeed, double followRadius, double positionBuffer, double rotationBuffer) {
        super(translation, rotation, movementSpeed, turnSpeed, followRadius, positionBuffer, rotationBuffer, null);
        super.setAction(generateEndAction());
    }

    /**
     * Constructs an EndWaypoint with the provided values.
     *
     * @param pose
     * @param movementSpeed  The speed in which the robot moves at while traversing this waypoint.
     * @param turnSpeed      The speed in which the robot turns at while traversing this waypoint.
     * @param followRadius   The distance in which the robot traverses this waypoint.
     * @param positionBuffer The expected level of error, the robot will consider itself at the waypoint when it is within the buffer. The buffer must be > 0.
     * @param rotationBuffer The expected level of error (in radians), the robot will consider itself at the waypoint when it is within the buffer. The buffer must be > 0.
     */
    public EndWaypoint(Pose2d pose, double movementSpeed, double turnSpeed, double followRadius, double positionBuffer, double rotationBuffer) {
        super(pose, movementSpeed, turnSpeed, followRadius, positionBuffer, rotationBuffer, null);
        super.setAction(generateEndAction());
    }

    /**
     * Constructs an EndWaypoint with the provided values.
     *
     * @param x              The x position of this waypoint.
     * @param y              The y position of this waypoint.
     * @param movementSpeed  The speed in which the robot moves at while traversing this waypoint.
     * @param turnSpeed      The speed in which the robot turns at while traversing this waypoint.
     * @param followRadius   The distance in which the robot traverses this waypoint.
     * @param positionBuffer The expected level of error, the robot will consider itself at the waypoint when it is within the buffer. The buffer must be > 0.
     * @param rotationBuffer The expected level of error (in radians), the robot will consider itself at the waypoint when it is within the buffer. The buffer must be > 0.
     */
    public EndWaypoint(double x, double y, double movementSpeed, double turnSpeed, double followRadius, double positionBuffer, double rotationBuffer) {
        super(x, y, movementSpeed, turnSpeed, followRadius, positionBuffer, rotationBuffer, null);
        super.setAction(generateEndAction());
    }

    /**
     * Constructs an EndWaypoint with the provided values.
     *
     * @param x               The x position of this waypoint.
     * @param y               The y position of this waypoint.
     * @param rotationRadians The preferred angle of this waypoint (in radians).
     * @param movementSpeed   The speed in which the robot moves at while traversing this waypoint.
     * @param turnSpeed       The speed in which the robot turns at while traversing this waypoint.
     * @param followRadius    The distance in which the robot traverses this waypoint.
     * @param positionBuffer  The expected level of error, the robot will consider itself at the waypoint when it is within the buffer. The buffer must be > 0.
     * @param rotationBuffer  The expected level of error (in radians), the robot will consider itself at the waypoint when it is within the buffer. The buffer must be > 0.
     */
    public EndWaypoint(double x, double y, double rotationRadians, double movementSpeed, double turnSpeed, double followRadius, double positionBuffer, double rotationBuffer) {
        super(x, y, rotationRadians, movementSpeed, turnSpeed, followRadius, positionBuffer, rotationBuffer, null);
        super.setAction(generateEndAction());
    }

    /**
     * Sets this endpoint as traversed.
     */
    @Override
    public void setTraversed() {
        isFinished = true;
    }

    /**
     * @return Whether this waypoint, and the path, is finished
     */
    public boolean isFinished() {
        return isFinished;
    }

    @Override
    public InterruptWaypoint setAction(InterruptAction action) {
        // not allowed to change the action
        throw new IllegalArgumentException("You cannot change the action of an end waypoint.");
    }

    /**
     * Creates and returns a default end action
     *
     * @return The created end action
     */
    private InterruptAction generateEndAction() {
        return () -> isFinished = true;
    }

    @Override
    public WaypointType getType() {
        return WaypointType.END;
    }

    @Override
    public void reset() {
        super.reset();
        isFinished = false;
    }

    @Override
    public String toString() {
        return String.format("EndWaypoint(%s, %s)", getTranslation().getX(), getTranslation().getY());
    }
}
