package com.lcrobotics.easyftclib.pathfinding;

import com.lcrobotics.easyftclib.pathfinding.types.WaypointType;
import com.lcrobotics.easyftclib.tools.geometry.Pose2d;

public interface Waypoint {
    /**
     * @return this Waypoint's type.
     */
    WaypointType getType();

    /**
     * @return this Waypoint's position.
     */
    Pose2d getPose();

    /**
     * @return the follow distance for this Waypoint.
     */
    double getFollowDistance();

    /**
     * @return the timeout period of this Waypoint.
     */
    long getTimeout();
}
