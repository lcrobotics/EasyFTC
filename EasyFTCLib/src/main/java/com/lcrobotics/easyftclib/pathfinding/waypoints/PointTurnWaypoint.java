package com.lcrobotics.easyftclib.pathfinding.waypoints;

import com.lcrobotics.easyftclib.pathfinding.types.WaypointType;
import com.lcrobotics.easyftclib.tools.geometry.Pose2d;
import com.lcrobotics.easyftclib.tools.geometry.Rotation2d;
import com.lcrobotics.easyftclib.tools.geometry.Translation2d;

public class PointTurnWaypoint extends GeneralWaypoint {

    // The expected level of error, the robot will consider itself at the waypoint
    // when it is within the buffer. The buffers must be > 0
    private double positionBuffer;
    private double rotationBuffer;

    // true if the robot has already passed this waypoint
    private boolean traversed = false;

    /**
     * Default constructor for a PointTurnWaypoint.
     */
    public PointTurnWaypoint() {
        positionBuffer = rotationBuffer = 0;
    }

    /**
     * Constructs a PointTurnWaypoint with the provided values.
     *
     * @param translation    The (x, y) translation of this waypoint.
     * @param rotation       The preferred angle of this waypoint.
     * @param movementSpeed  The speed in which the robot moves at while traversing this waypoint.
     * @param turnSpeed      The speed in which the robot turns at while traversing this waypoint.
     * @param followRadius   The distance in which the robot traverses this waypoint.
     * @param positionBuffer The expected level of error, the robot will consider itself at the waypoint when it is within the buffer. The buffer must be > 0.
     * @param rotationBuffer The expected level of error (in radians), the robot will consider itself at the waypoint when it is within the buffer. The buffer must be > 0.
     */
    public PointTurnWaypoint(Translation2d translation, Rotation2d rotation,
                             double movementSpeed, double turnSpeed,
                             double followRadius,
                             double positionBuffer, double rotationBuffer) {

        super(translation, rotation, movementSpeed, turnSpeed, followRadius);
        this.positionBuffer = positionBuffer;
        this.rotationBuffer = rotationBuffer;

    }

    /**
     * Constructs a PointTurnWaypoint with the provided values.
     *
     * @param pose           Position and preferred angle of this waypoint.
     * @param movementSpeed  The speed in which the robot moves at while traversing this waypoint
     * @param turnSpeed      The speed in which the robot turns at while traversing this waypoint.
     * @param followRadius   The distance in which the robot traverses this waypoint.
     * @param positionBuffer The expected level of error, the robot will consider itself at the waypoint when it is within the buffer. The buffer must be > 0.
     * @param rotationBuffer The expected level of error (in radians), the robot will consider itself at the waypoint when it is within the buffer. The buffer must be > 0.
     */
    public PointTurnWaypoint(Pose2d pose, double movementSpeed, double turnSpeed, double followRadius, double positionBuffer, double rotationBuffer) {
        super(pose, movementSpeed, turnSpeed, followRadius);
        this.positionBuffer = positionBuffer;
        this.rotationBuffer = rotationBuffer;
    }

    /**
     * Constructs a PointTurnWaypoint with the provided values.
     *
     * @param x              The x position of this waypoint.
     * @param y              The y position of this waypoint.
     * @param movementSpeed  The speed in which the robot moves at while traversing this waypoint.
     * @param turnSpeed      The speed in which the robot turns at while traversing this waypoint.
     * @param followRadius   The distance in which the robot traverses this waypoint.
     * @param positionBuffer The expected level of error, the robot will consider itself at the waypoint when it is within the buffer. The buffer must be > 0.
     * @param rotationBuffer The expected level of error (in radians), the robot will consider itself at the waypoint when it is within the buffer. The buffer must be > 0.
     */
    public PointTurnWaypoint(double x, double y, double movementSpeed, double turnSpeed, double followRadius, double positionBuffer, double rotationBuffer) {
        super(x, y, movementSpeed, turnSpeed, followRadius);
        this.positionBuffer = positionBuffer;
        this.rotationBuffer = rotationBuffer;
    }

    /**
     * Constructs a PointTurnWaypoint with the provided values.
     *
     * @param x               The x position of this waypoint.
     * @param y               The y position of this waypoint.
     * @param rotationRadians The preferred angle of this waypoint (in radians).
     * @param movementSpeed   The speed in which the robot moves at while traversing this waypoint..
     * @param turnSpeed       The speed in which the robot turns at while traversing this waypoint
     * @param followRadius    The distance in which the robot traverses this waypoint.
     * @param positionBuffer  The expected level of error, the robot will consider itself at the waypoint when it is within the buffer. The buffer must be > 0.
     * @param rotationBuffer  The expected level of error (in radians), the robot will consider itself at the waypoint when it is within the buffer. The buffer must be > 0.
     */
    public PointTurnWaypoint(double x, double y, double rotationRadians, double movementSpeed, double turnSpeed, double followRadius, double positionBuffer, double rotationBuffer) {
        super(x, y, rotationRadians, movementSpeed, turnSpeed, followRadius);
        this.positionBuffer = positionBuffer;
        this.rotationBuffer = rotationBuffer;
    }

    /**
     * @return The position buffer.
     */
    public double getPositionBuffer() {
        return positionBuffer;
    }

    /**
     * Sets this waypoint's position buffer.
     *
     * @param buffer Position buffer to be set.
     * @return This PointTurnWaypoint, used for chaining methods.
     */
    public PointTurnWaypoint setPositionBuffer(double buffer) {
        positionBuffer = buffer;
        return this;
    }

    /**
     * @return The rotation buffer.
     */
    public double getRotationBuffer() {
        return rotationBuffer;
    }

    /**
     * Sets this waypoint's rotation buffer.
     *
     * @param buffer Rotation buffer to be set.
     * @return This PointTurnWaypoint, used for chaining methods.
     */
    public PointTurnWaypoint setRotationBuffer(double buffer) {
        rotationBuffer = buffer;
        return this;
    }

    /**
     * @return If this waypoint has already been traversed.
     */
    public boolean hasTraversed() {
        return traversed;
    }

    /**
     * Tells the waypoint that it has been traversed.
     */
    public void setTraversed() {
        traversed = true;
    }

    /**
     * Verified the buffer it valid. The buffer is valid if it is >1.
     *
     * @param buffer Buffer to be checked.
     * @return The buffer.
     * @throws IllegalArgumentException If the buffer is not valid.
     */
    private double verifyBuffer(double buffer) {
        if (buffer <= 0)
            throw new IllegalArgumentException("The buffer must be > 0");
        return buffer;
    }

    @Override
    public void reset() {
        traversed = false;
    }

    @Override
    public WaypointType getType() {
        return WaypointType.POINT_TURN;
    }

    @Override
    public String toString() {
        return String.format("PointTurnWaypoint(%s, %s)", getTranslation().getX(), getTranslation().getY());
    }
}
