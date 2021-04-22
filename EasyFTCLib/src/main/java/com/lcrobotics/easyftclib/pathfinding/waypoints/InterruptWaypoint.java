package com.lcrobotics.easyftclib.pathfinding.waypoints;

import com.lcrobotics.easyftclib.pathfinding.actions.InterruptAction;
import com.lcrobotics.easyftclib.pathfinding.types.WaypointType;
import com.lcrobotics.easyftclib.tools.geometry.Pose2d;
import com.lcrobotics.easyftclib.tools.geometry.Rotation2d;
import com.lcrobotics.easyftclib.tools.geometry.Translation2d;

public class InterruptWaypoint extends PointTurnWaypoint {
    // action to perform at this waypoint
    private InterruptAction action;

    // Whether the action has already been done
    private boolean actionPerformed = false;

    /**
     * Default constructor for a InterruptWaypoint.
     */
    public InterruptWaypoint() {
        action = () -> {
            // default is nothing
        };
    }

    /**
     * Constructs an InterruptWaypoint with the provided values.
     *
     * @param translation    The (x, y) translation of this waypoint.
     * @param rotation       The rotation (preferred angle) of this waypoint.
     * @param movementSpeed  The speed in which the robot moves at while traversing this waypoint.
     * @param turnSpeed      The speed in which the robot turns at while traversing this waypoint.
     * @param followRadius   The distance in which the robot traverses this waypoint.
     * @param positionBuffer The expected level of error, the robot will consider itself at the waypoint when it is within the buffer. The buffer must be > 0.
     * @param rotationBuffer The expected level of error (in radians), the robot will consider itself at the waypoint when it is within the buffer. The buffer must be > 0.
     * @param action         The action the robot performs at this waypoint.
     */
    public InterruptWaypoint(Translation2d translation, Rotation2d rotation, double movementSpeed, double turnSpeed, double followRadius, double positionBuffer, double rotationBuffer, InterruptAction action) {
        super(translation, rotation, movementSpeed, turnSpeed, followRadius, positionBuffer, rotationBuffer);
        this.action = action;
    }

    /**
     * Constructs an InterruptWaypoint with the provided values.
     *
     * @param pose
     * @param movementSpeed  The speed in which the robot moves at while traversing this waypoint.
     * @param turnSpeed      The speed in which the robot turns at while traversing this waypoint.
     * @param followRadius   The distance in which the robot traverses this waypoint.
     * @param positionBuffer The expected level of error, the robot will consider itself at the waypoint when it is within the buffer. The buffer must be > 0.
     * @param rotationBuffer The expected level of error (in radians), the robot will consider itself at the waypoint when it is within the buffer. The buffer must be > 0.
     * @param action         The action the robot performs at this waypoint.
     */
    public InterruptWaypoint(Pose2d pose, double movementSpeed, double turnSpeed, double followRadius, double positionBuffer, double rotationBuffer, InterruptAction action) {
        super(pose, movementSpeed, turnSpeed, followRadius, positionBuffer, rotationBuffer);
        this.action = action;
    }

    /**
     * Constructs an InterruptWaypoint with the provided values.
     *
     * @param x              The x position of this waypoint.
     * @param y              The y position of this waypoint.
     * @param movementSpeed  The speed in which the robot moves at while traversing this waypoint.
     * @param turnSpeed      The speed in which the robot turns at while traversing this waypoint.
     * @param followRadius   The distance in which the robot traverses this waypoint.
     * @param positionBuffer The expected level of error, the robot will consider itself at the waypoint when it is within the buffer. The buffer must be > 0.
     * @param rotationBuffer The expected level of error (in radians), the robot will consider itself at the waypoint when it is within the buffer. The buffer must be > 0.
     * @param action         The action the robot performs at this point.
     */
    public InterruptWaypoint(double x, double y, double movementSpeed, double turnSpeed, double followRadius, double positionBuffer, double rotationBuffer, InterruptAction action) {
        super(x, y, movementSpeed, turnSpeed, followRadius, positionBuffer, rotationBuffer);
        this.action = action;
    }

    /**
     * Constructs an InterruptWaypoint with the provided values.
     *
     * @param x               The x position of this waypoint.
     * @param y               The y position of this waypoint.
     * @param rotationRadians The preferred angle of this waypoint (in radians).
     * @param movementSpeed   The speed in which the robot moves at while traversing this waypoint.
     * @param turnSpeed       The speed in which the robot turns at while traversing this waypoint.
     * @param followRadius    The distance in which the robot traverses this waypoint.
     * @param positionBuffer  The expected level of error, the robot will consider itself at the waypoint when it is within the buffer. The buffer must be > 0.
     * @param rotationBuffer  The expected level of error (in radians), the robot will consider itself at the waypoint when it is within the buffer. The buffer must be > 0.
     * @param action          The action the robot performs at this point.
     */
    public InterruptWaypoint(double x, double y, double rotationRadians, double movementSpeed, double turnSpeed, double followRadius, double positionBuffer, double rotationBuffer, InterruptAction action) {
        super(x, y, rotationRadians, movementSpeed, turnSpeed, followRadius, positionBuffer, rotationBuffer);
        this.action = action;
    }

    /**
     * Sets the action of this InterruptWaypoint.
     *
     * @param action Action to be set.
     * @return This InterruptWaypoint, used for chaining methods.
     */
    public InterruptWaypoint setAction(InterruptAction action) {
        this.action = action;
        return this;
    }


    /**
     * If the action has not already been performed, performs the action.
     */
    public void performAction() {
        if (!actionPerformed && action != null) {
            action.doAction();
            actionPerformed = true;
        }
    }

    /**
     * @return Whether the action has already been performed
     */
    public boolean actionPerformed() {
        return actionPerformed;
    }

    @Override
    public void reset() {
        super.reset();
        actionPerformed = false;
    }

    @Override
    public WaypointType getType() {
        return WaypointType.INTERRUPT;
    }

    @Override
    public String toString() {
        return String.format("InterruptWaypoint(%s, %s)", getTranslation().getX(), getTranslation().getY());
    }
}
