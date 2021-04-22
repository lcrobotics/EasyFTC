package com.lcrobotics.easyftclib.pathfinding.waypoints;

import com.lcrobotics.easyftclib.pathfinding.Waypoint;
import com.lcrobotics.easyftclib.pathfinding.types.WaypointType;
import com.lcrobotics.easyftclib.tools.MathUtils;
import com.lcrobotics.easyftclib.tools.geometry.Pose2d;
import com.lcrobotics.easyftclib.tools.geometry.Rotation2d;
import com.lcrobotics.easyftclib.tools.geometry.Translation2d;

public class GeneralWaypoint extends Pose2d implements Waypoint {

    // if running longer than the timeout, path will be stopped
    private long timeoutMilliseconds;

    // move speed of robot, from 0 to 1
    private double movementSpeed;

    // turn speed of robot, from 0 to 1
    private double turnSpeed;

    // radius of pure pursuit circle
    private double followRadius;

    // if this waypoint uses its own angle
    private boolean usePreferredAngle;
    private double preferredAngle;

    // if this waypoint inherits config from previous node
    private boolean copyMode;

    /**
     * Default constructor for a GeneralWaypoint
     */
    public GeneralWaypoint() {
        movementSpeed = turnSpeed = followRadius = 0;
        timeoutMilliseconds = -1;
        usePreferredAngle = copyMode = false;
    }

    public GeneralWaypoint(double x, double y) {
        this(x, y, 0, 0, 0);
        copyMode = true;
    }

    /**
     * Constructs a GeneralWaypoint with the given values.
     *
     * @param translation   The (x, y) translation of this waypoint.
     * @param rotation      The preferred angle for this waypoint.
     * @param movementSpeed The speed that the robot moves while traversing this waypoint.
     * @param turnSpeed     The speed that the robot turns while traversing this waypoint.
     * @param followRadius  The distance at which the robot follows this waypoint.
     */
    public GeneralWaypoint(Translation2d translation, Rotation2d rotation, double movementSpeed, double turnSpeed, double followRadius) {
        super(translation, rotation);

        this.movementSpeed = normalizeSpeed(movementSpeed);
        this.turnSpeed = normalizeSpeed(turnSpeed);
        this.followRadius = followRadius;

        timeoutMilliseconds = -1;
        usePreferredAngle = false;
        preferredAngle = 0;
        copyMode = false;
    }

    /**
     * Constructs a GeneralWaypoint with the given values.
     *
     * @param pose          Position and preferred angle of this waypoint.
     * @param movementSpeed The speed that the robot moves while traversing this waypoint.
     * @param turnSpeed     The speed that the robot turns while traversing this waypoint.
     * @param followRadius  The distance at which the robot follows this waypoint.
     */
    public GeneralWaypoint(Pose2d pose, double movementSpeed, double turnSpeed, double followRadius) {
        this(pose.getTranslation(), pose.getRotation(), movementSpeed, turnSpeed, followRadius);
    }

    /**
     * Constructs a GeneralWaypoint with the given values.
     *
     * @param x             The x position of this waypoint.
     * @param y             The y position of this waypoint.
     * @param movementSpeed The speed that the robot moves while traversing this waypoint.
     * @param turnSpeed     The speed that the robot turns while traversing this waypoint.
     * @param followRadius  The distance at which the robot follows this waypoint.
     */
    public GeneralWaypoint(double x, double y, double movementSpeed, double turnSpeed, double followRadius) {
        this(new Translation2d(x, y), new Rotation2d(), movementSpeed, turnSpeed, followRadius);
    }

    /**
     * Constructs a GeneralWaypoint with the given values.
     *
     * @param x               The x position of this waypoint.
     * @param y               The y position of this waypoint.
     * @param rotationRadians The preferred angle (in radians) of this waypoint.
     * @param movementSpeed   The speed that the robot moves while traversing this waypoint.
     * @param turnSpeed       The speed that the robot turns while traversing this waypoint.
     * @param followRadius    The distance at which the robot follows this waypoint.
     */
    public GeneralWaypoint(double x, double y, double rotationRadians, double movementSpeed, double turnSpeed, double followRadius) {
        this(new Translation2d(x, y), new Rotation2d(rotationRadians), movementSpeed, turnSpeed, followRadius);
    }

    /**
     * @return the movement speed of this waypoint.
     */
    public double getMovementSpeed() {
        return movementSpeed;
    }

    /**
     * Sets the movement speed of this waypoint.
     *
     * @param movementSpeed Speed to set.
     * @return this GeneralWaypoint, used for chaining methods.
     */
    public GeneralWaypoint setMovementSpeed(double movementSpeed) {
        this.movementSpeed = movementSpeed;
        return this;
    }

    /**
     * @return the turn speed of this waypoint.
     */
    public double getTurnSpeed() {
        return turnSpeed;
    }

    /**
     * Sets the turn speed of this waypoint.
     *
     * @param turnSpeed Speed to be set.
     * @return this GeneralWaypoint, used for chaining methods.
     */
    public GeneralWaypoint setTurnSpeed(double turnSpeed) {
        this.turnSpeed = turnSpeed;
        return this;
    }

    /**
     * @return the follow radius of this waypoint.
     */
    public double getFollowRadius() {
        return followRadius;
    }

    /**
     * Sets the follow radius of this waypoint.
     *
     * @param followRadius Radius to be set.
     * @return this waypoint, used for chaining methods.
     */
    public GeneralWaypoint setFollowRadius(double followRadius) {
        this.followRadius = followRadius;
        return this;
    }

    /**
     * @return this waypoint's preferred angle (in radians).
     * @throws IllegalStateException If no preferred angle is used.
     */
    public double getPreferredAngle() {
        if (usePreferredAngle)
            return preferredAngle;
        throw new IllegalStateException("This waypoint is not using a preferred angle");
    }

    /**
     * Sets and enables this waypoint's preferred angle.
     *
     * @param angle Angle to be set.
     * @return this waypoint, used for chaining methods.
     */
    public GeneralWaypoint setPreferredAngle(double angle) {
        usePreferredAngle = true;
        preferredAngle = angle;
        return this;
    }

    /**
     * @return whether this waypoint uses a preferred angle.
     */
    public boolean usingPreferredAngle() {
        return usePreferredAngle;
    }

    /**
     * Disables this waypoint's preferredAngle. This is disabled by default.
     *
     * @return This GeneralWaypoint, used for chaining methods.
     */
    public GeneralWaypoint disablePreferredAngle() {
        usePreferredAngle = false;
        preferredAngle = 0;
        return this;
    }

    /**
     * Resets this waypoint. This is called by Path.
     */
    public void reset() {
        // GeneralWaypoints don't have anything to reset.
    }

    /**
     * Clamps the given speed into the range [0, 1].
     *
     * @param speed The raw speed to be normalized.
     * @return Normalized value.
     */
    protected double normalizeSpeed(double speed) {
        return MathUtils.clamp(speed, 0, 1);
    }

    /**
     * Copies configuration from the given waypoint. Does nothing if copy mode is disabled.
     *
     * @param waypoint Waypoint to copy.
     * @throws IllegalArgumentException If the waypoint provided isn't a GeneralWaypoint
     */
    public void inherit(Waypoint waypoint) {
        if (!copyMode)
            return;
        if (!(waypoint instanceof GeneralWaypoint))
            throw new IllegalArgumentException("A " + getType() + " waypoint cannot inherit the configuration of a " + waypoint.getType() + " waypoint");

        GeneralWaypoint w = (GeneralWaypoint) waypoint;

        this.movementSpeed = w.movementSpeed;
        this.turnSpeed = w.turnSpeed;
        this.followRadius = w.followRadius;
        this.timeoutMilliseconds = w.timeoutMilliseconds;

        if (w.usePreferredAngle)
            this.preferredAngle = w.preferredAngle;

        this.usePreferredAngle = w.usePreferredAngle;
    }

    @Override
    public WaypointType getType() {
        return WaypointType.GENERAL;
    }

    @Override
    public Pose2d getPose() {
        return this;
    }

    @Override
    public double getFollowDistance() {
        return followRadius;
    }

    @Override
    public long getTimeout() {
        return timeoutMilliseconds;
    }

    /**
     * Sets the timeout period of this waypoint. This is optional.
     *
     * @param timeoutMilliseconds The timeout period of this waypoint.
     * @return This GeneralWaypoint, used for chaining methods.
     */
    public GeneralWaypoint setTimeout(long timeoutMilliseconds) {
        this.timeoutMilliseconds = timeoutMilliseconds;
        return this;
    }

    @Override
    public String toString() {
        return String.format("GeneralWaypoint(%s, %s)", getTranslation().getX(), getTranslation().getY());
    }
}
