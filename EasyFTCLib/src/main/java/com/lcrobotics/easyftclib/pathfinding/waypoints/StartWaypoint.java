package com.lcrobotics.easyftclib.pathfinding.waypoints;

import com.lcrobotics.easyftclib.pathfinding.Waypoint;
import com.lcrobotics.easyftclib.pathfinding.types.WaypointType;
import com.lcrobotics.easyftclib.tools.geometry.Pose2d;
import com.lcrobotics.easyftclib.tools.geometry.Rotation2d;
import com.lcrobotics.easyftclib.tools.geometry.Translation2d;

public class StartWaypoint extends Pose2d implements Waypoint {

    private long timeoutMilliseconds;

    /**
     * Default constructor for a StartWaypoint.
     */
    public StartWaypoint() {
        timeoutMilliseconds = -1;
    }

    /**
     * Construct a StartWaypoint located at the provided translation.
     *
     * @param translation Position (x, y) of this waypoint.
     */
    public StartWaypoint(Translation2d translation) {
        super(translation, new Rotation2d(0));
    }

    /**
     * Construct a StartWaypoint located at the provided pose.
     *
     * @param pose Position (x, y) of this waypoint.
     */
    public StartWaypoint(Pose2d pose) {
        super(pose.getTranslation(), pose.getRotation());
    }

    /**
     * Construct a StartWaypoint located at the provided coordinate.
     *
     * @param x X Position of this waypoint.
     * @param y Y Position of this waypoint.
     */
    public StartWaypoint(double x, double y) {
        super(x, y, new Rotation2d(0));
    }

    @Override
    public WaypointType getType() {
        return WaypointType.START;
    }

    @Override
    public Pose2d getPose() {
        return this;
    }

    @Override
    public double getFollowDistance() {
        return 0;
    }

    @Override
    public long getTimeout() {
        return timeoutMilliseconds;
    }

    public void setTimeout(long timeoutMilliseconds) {
        this.timeoutMilliseconds = timeoutMilliseconds;
    }
    @Override
    public String toString() {
        return String.format("StartWaypoint(%s, %s)", getTranslation().getX(), getTranslation().getY());
    }
}
