package com.lcrobotics.easyftclib.pathfinding;

import android.support.annotation.NonNull;

import com.lcrobotics.easyftclib.CommandCenter.driveTrain.MecanumDrive;
import com.lcrobotics.easyftclib.encoderOdometry.Odometry;
import com.lcrobotics.easyftclib.pathfinding.actions.TriggeredAction;
import com.lcrobotics.easyftclib.pathfinding.types.PathType;
import com.lcrobotics.easyftclib.pathfinding.types.WaypointType;
import com.lcrobotics.easyftclib.pathfinding.waypoints.EndWaypoint;
import com.lcrobotics.easyftclib.pathfinding.waypoints.GeneralWaypoint;
import com.lcrobotics.easyftclib.pathfinding.waypoints.InterruptWaypoint;
import com.lcrobotics.easyftclib.pathfinding.waypoints.PointTurnWaypoint;
import com.lcrobotics.easyftclib.tools.MathUtils;
import com.lcrobotics.easyftclib.tools.geometry.Pose2d;
import com.lcrobotics.easyftclib.tools.geometry.Rotation2d;
import com.lcrobotics.easyftclib.tools.geometry.Translation2d;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.LinkedList;
import java.util.List;
import java.util.Queue;

public class Path extends ArrayList<Waypoint> {
    private static final double[] STOP = {0, 0, 0};
    private static PathMotionProfile defaultMotionProfile = null;
    // action lists
    private final List<TriggeredAction> triggeredActions;
    private final Queue<InterruptWaypoint> interruptActionQueue;
    // path type
    private PathType pathType;
    // motion profile
    private PathMotionProfile motionProfile;
    // timeout stuff
    private long timeoutMilliseconds;
    private long startTime;
    private Waypoint lastWaypoint;
    private long lastWaypointTimestamp;
    private boolean timedOut;
    private boolean retraceEnabled;
    private boolean initComplete;
    // retracing
    private boolean retracing;
    private double retraceMovementSpeed;
    private double retraceTurnSpeed;
    private Translation2d lastKnownIntersection;

    /**
     * Constructs an empty path. Use add() to add waypoints.
     */
    public Path() {
        this(new Waypoint[]{});
    }

    /**
     * Constructs a path with the given waypoints.
     *
     * @param waypoints Waypoints in the path.
     */
    public Path(Waypoint... waypoints) {
        addAll(Arrays.asList(waypoints));
        pathType = PathType.WAYPOINT_ORDERING_CONTROLLED;
        timeoutMilliseconds = -1;
        startTime = -1;
        lastWaypointTimestamp = 0;
        retraceMovementSpeed = 1;
        retraceTurnSpeed = 1;
        retraceEnabled = true;
        initComplete = false;
        timedOut = false;
        triggeredActions = new ArrayList<>();
        interruptActionQueue = new LinkedList<>();
        motionProfile = getDefaultMotionProfile();
        lastWaypoint = null;
    }

    /**
     * Initiates the path. This must be ran before using the path. This checks to make sure everything in the path is
     * valid. Use isLegalPath() to check if a path is valid.
     *
     * @throws IllegalStateException If the path is not legal.
     */
    public void init() {
        // verify path
        verify();
        reset();
        // Configure unconfigured waypoints.
        for (int i = 1; i < size(); i++) {
            ((GeneralWaypoint) get(i)).inherit(get(i - 1));
        }

        initComplete = true;
    }

    /**
     * Retraces the robot back to last known intersection of the path.
     *
     * @param robotPos Position and heading of the robot
     * @return An array containing the strafe, drive, and turn motor powers (in that order).
     */
    private double[] retrace(Pose2d robotPos) {
        // move to last known intersection
        double[] motorPowers = PathfindingUtil.moveToPosition(
                robotPos,
                new Pose2d(lastKnownIntersection, new Rotation2d(robotPos.getHeading())),
                false);

        motorPowers[0] *= retraceMovementSpeed;
        motorPowers[1] *= retraceMovementSpeed;
        motorPowers[2] *= retraceTurnSpeed;
        return motorPowers;
    }

    /**
     * Initiates the automatic path following feature. The robot will follow the path and perform actions as
     * configured.
     *
     * @param mecanumDrive The robot's drive base. Only mecanum drives are supported currently.
     * @param odometry     The robot's odometry.
     * @return True if the path completed successfully, false if the path did not (timed out, lost path, etc.).
     */
    public boolean followPath(@NonNull MecanumDrive mecanumDrive, @NonNull Odometry odometry) {
        // Init the path
        init();

        while (!isFinished()) {
            // get current position
            Pose2d robotPos = odometry.getPose();

            // get motor powers from loop
            double[] motorPowers = loop(robotPos);
            // update motor powers
            mecanumDrive.driveRobotCentric(motorPowers[0], motorPowers[1], motorPowers[2]);
            // make sure loop didn't return {0, 0, 0}
            if (!isFinished()) {
                boolean pathAborted = Arrays.stream(motorPowers).allMatch(p -> p == 0);

                if (pathAborted) {
                    return false;
                }
            }
            odometry.updatePose();
        }
        // after the path is done, stop the motors
        mecanumDrive.stop();
        return true;
    }

    /**
     * This is the principle path method. After everything is configured and initiated, this method can be used. Using
     * the robot's horizontal, y, and rotation, this method calculates the appropriate motor powers for the robot to
     * follow the path. This method calls all triggered/interrupted actions automatically. If this returns zero motor
     * speeds {0, 0, 0} that means the path has either (1) timed out, (2) lost the path and retrace was disabled, or (3)
     * reached the destination. Use isFinished() and timedOut() to troubleshoot.
     *
     * @param robotPos Position and angle of robot
     * @return A double array containing the motor powers. a[0] is the x power, a[1] is the y power, and a[2] is the
     * turn power.
     */
    public double[] loop(Pose2d robotPos) {
        // check that init() has been called first
        if (!initComplete) {
            throw new IllegalStateException("Must call init() method before calling loop()");
        }
        // return no robot speeds if the path has timed out
        if (timedOut) {
            return STOP;
        }

        // check if path has timed out
        if (timeoutMilliseconds != -1) {
            if (startTime == -1) {
                startTime = System.currentTimeMillis();
            } else if (startTime + timeoutMilliseconds < System.currentTimeMillis()) {
                // path has timed out
                timedOut = true;

                return STOP;
            }
        }
        // perform all actions
        loopTriggeredActions();
        runQueuedInterruptActions();
        // get all intersections
        List<TaggedIntersection> intersections = new ArrayList<>();

        for (int i = 1; i < size(); i++) {
            // get path line and follow circle intersections
            Translation2d linePoint1 = get(i - 1).getPose().getTranslation();
            Translation2d linePoint2 = get(i).getPose().getTranslation();
            List<Translation2d> points = PathfindingUtil.lineCircleIntersection(robotPos.getTranslation(),
                    get(i).getFollowDistance(), linePoint1, linePoint2);

            for (Translation2d point : points) {
                // add results to list
                intersections.add(new TaggedIntersection(point, get(i), i));
            }
            if (get(i) instanceof PointTurnWaypoint) {
                // if waypoint is a point turn waypoint, decrease the follow radius so the next point is always found
                double adjustedRadius = Math.hypot(linePoint2.getX() - robotPos.getX(),
                        linePoint2.getY() - robotPos.getY()) - 1e-9;
                if (adjustedRadius < get(i).getFollowDistance()) {
                    // add intersection
                    intersections.add(new TaggedIntersection(((PointTurnWaypoint) get(i)).getTranslation(), get(i), i));
                }
            }
            // now we have all intersections
        }
        // if no intersections are found, we have lost the path. retrace to find it again

        if (intersections.size() == 0) {
            if (retracing) {
                return retrace(robotPos);
            }
            if (retraceEnabled) {
                // try to re-find path
                if (lastKnownIntersection == null) {
                    lastKnownIntersection = get(0).getPose().getTranslation();
                }
                retracing = true;
                return retrace(robotPos);
            } else {
                return STOP;
            }
        } else {
            retracing = false;
        }
        // intersections are handled differently depending on the path type
        TaggedIntersection bestIntersection = intersections.get(0);
        if (pathType == PathType.HEADING_CONTROLLED) {
            bestIntersection = selectHeadingControlledIntersection(intersections, robotPos);
        } else {
            bestIntersection = selectWaypointOrderingControlledIntersection(intersections);
        }
        // store last intersection for future retracing
        if (retraceEnabled) {
            lastKnownIntersection = bestIntersection.intersection;
        }

        // update timeout values if this intersection is with a new waypoint
        if (bestIntersection.taggedPoint != lastWaypoint) {
            lastWaypoint = bestIntersection.taggedPoint;
            lastWaypointTimestamp = System.currentTimeMillis();
        }
        // make sure waypoint hasn't timed out
        if (lastWaypoint.getTimeout() != -1 &&
                lastWaypointTimestamp + lastWaypoint.getTimeout() < System.currentTimeMillis()) {

            timedOut = true;
            return STOP;
        }
        // handle intersection based on what type of waypoint it's associated with
        double[] motorPowers;
        switch (lastWaypoint.getType()) {
            case GENERAL:
                motorPowers = handleGeneralIntersection(bestIntersection, robotPos);
                break;
            case POINT_TURN:
                motorPowers = handlePointTurnIntersection(bestIntersection, robotPos);
                break;
            case INTERRUPT:
                motorPowers = handleInterruptIntersection(bestIntersection, robotPos);
                break;
            case END:
                motorPowers = handleEndIntersection(bestIntersection, robotPos);
                break;
            case START:
                // This should never happen.
                throw new IllegalStateException("Path has lost integrity.");
            default:
                motorPowers = STOP;
                break;
        }
        // adjust speeds
        adjustSpeedWithProfile(motorPowers, bestIntersection, robotPos.getTranslation());
        normalizeMotorSpeeds(motorPowers);

        return motorPowers;
    }

    /**
     * Selects and returns the "best" intersection from the given list using heading control. The intersection is chosen
     * based on the following rules: 1. If the list contains any untraversed waypoints, they are given priority and the
     * best intersection is the point closest to the point turn waypoint. 2. If the list contains no point turn points,
     * then it chooses the intersection the robot is oriented most closely towards.
     *
     * @param intersections Intersection list.
     * @param robotPos      Robot's current position/rotation.
     * @return The best intersection in the form of a TaggedIntersection.
     */
    private TaggedIntersection selectHeadingControlledIntersection(List<TaggedIntersection> intersections,
            Pose2d robotPos) {
        TaggedIntersection bestIntersection = intersections.get(0);
        boolean pointTurnPriority = false;
        /*
          In a heading controlled path, the intersection the robot is most closely oriented
          toward is considered the "best point".
         */
        for (TaggedIntersection intersection : intersections) {
            // check to see if point turn waypoint is found
            if (intersection.taggedPoint instanceof PointTurnWaypoint) {
                PointTurnWaypoint ptWaypoint = (PointTurnWaypoint) intersection.taggedPoint;

                if (!ptWaypoint.hasTraversed()) {
                    // give point turn waypoints that haven't been traversed yet priority
                    pointTurnPriority = true;
                    if (!(bestIntersection.taggedPoint instanceof PointTurnWaypoint)) {
                        bestIntersection = intersection;
                    } else {
                        // if two intersection are found with a pt waypoint, choose the one
                        // closer to the waypoint
                        if (bestIntersection.waypointIndex < intersection.waypointIndex) {
                            bestIntersection = intersection;
                        } else if (bestIntersection.waypointIndex == intersection.waypointIndex) {
                            // check if it is in front
                            if (PathfindingUtil.isInFront(get(
                                    intersection.waypointIndex - 1).getPose().getTranslation(),
                                    intersection.taggedPoint.getPose().getTranslation(),
                                    intersection.intersection,
                                    bestIntersection.intersection)) {

                                bestIntersection = intersection;
                            }
                        }
                    }
                }
            } else if (!pointTurnPriority) {
                // normal case
                // least relative angle is best
                double absoluteAngleToIntersection = Math.atan2(intersection.intersection.getY(),
                        intersection.intersection.getX());
                double relativeAngleToIntersection =
                        absoluteAngleToIntersection - robotPos.getHeading();

                double absoluteAngleToBestIntersection = Math.atan2(
                        bestIntersection.intersection.getY(), bestIntersection.intersection.getX());
                double relativeAngleToBestIntersection =
                        absoluteAngleToBestIntersection - robotPos.getHeading();

                if (relativeAngleToIntersection < relativeAngleToBestIntersection) {
                    bestIntersection = intersection;
                }
            }
        }

        return bestIntersection;
    }

    /**
     * Selects and returns the "best" intersection from the given list by choosing the intersection that is farthest
     * along the path. The intersection is chosen based on the following rules: 1. If the list contains any untraversed
     * waypoints, they are given priority and the best intersection is the point closest to the point turn waypoint. 2.
     * If the list contains no point turn points, then it chooses the intersection that is farthest along the path.
     *
     * @param intersections Intersection list.
     * @return The best intersection in the form of a TaggedIntersection.
     */
    private TaggedIntersection selectWaypointOrderingControlledIntersection(List<TaggedIntersection> intersections) {
        TaggedIntersection bestIntersection = intersections.get(0);
        boolean pointTurnPriority = false;

        /*
         * In a waypoint ordering controlled path,
         * the best intersection is the one farthest along the path
         */
        for (TaggedIntersection intersection : intersections) {
            // check to see if a point turn waypoint is found.
            if (intersection.taggedPoint instanceof PointTurnWaypoint) {
                PointTurnWaypoint ptWaypoint = (PointTurnWaypoint) intersection.taggedPoint;
                if (!ptWaypoint.hasTraversed()) {
                    // If point turn waypoint is found, and it has not already been traversed,
                    // then it takes priority.
                    pointTurnPriority = true;
                    if (!(bestIntersection.taggedPoint instanceof PointTurnWaypoint)) {
                        bestIntersection = intersection;
                    } else if (((PointTurnWaypoint) bestIntersection.taggedPoint).hasTraversed()) {
                        bestIntersection = intersection;
                    } else {
                        // If two intersections associated with a point turn waypoint are found,
                        // choose the one closer to the waypoint.
                        if (bestIntersection.waypointIndex > intersection.waypointIndex
                                || ptWaypoint.hasTraversed()) {
                            // intersection is obviously behind
                            bestIntersection = intersection;
                        } else if (bestIntersection.waypointIndex == intersection.waypointIndex) {
                            // check to see if it is in front on the path
                            if (PathfindingUtil.isInFront(get(
                                    intersection.waypointIndex - 1).getPose().getTranslation(),
                                    intersection.taggedPoint.getPose().getTranslation(),
                                    intersection.intersection,
                                    bestIntersection.intersection)) {

                                bestIntersection = intersection;
                            }
                        }
                    }
                }
            } else if (!pointTurnPriority) {
                // normal case
                if (bestIntersection.waypointIndex < intersection.waypointIndex) {
                    // intersection is obviously ahead
                    bestIntersection = intersection;
                } else if (bestIntersection.waypointIndex == intersection.waypointIndex) {
                    // check if it is ahead on the path
                    if (PathfindingUtil.isInFront(get(
                            intersection.waypointIndex - 1).getPose().getTranslation(),
                            intersection.taggedPoint.getPose().getTranslation(),
                            intersection.intersection,
                            bestIntersection.intersection)) {

                        bestIntersection = intersection;
                    }
                }
            }
        }
        return bestIntersection;
    }

    /**
     * Returns the motor speeds required to approach the given intersection.
     *
     * @param intersection Intersection to approach.
     * @param robotPos     Robot's current position/rotation.
     * @return Final motor speeds.
     */
    private double[] handleGeneralIntersection(TaggedIntersection intersection, Pose2d robotPos) {
        /*
         * General intersections are handled like normal pure pursuit intersections. The robot simply moves towards
         * them.
         */
        GeneralWaypoint waypoint = (GeneralWaypoint) intersection.taggedPoint;

        double targetAngle = waypoint.usingPreferredAngle() ? waypoint.getPreferredAngle() :
                             Math.atan2(intersection.intersection.getY() - robotPos.getY(),
                                     intersection.intersection.getX() - robotPos.getX());

        return PathfindingUtil.moveToPosition(robotPos,
                new Pose2d(intersection.intersection, new Rotation2d(targetAngle)), false);
    }

    /**
     * Returns the motor speeds required to approach the given point turn intersection. This will cause the robot to
     * behave as follows: 1. Approach and decelerate to the waypoint. 2. Perform a point turn. 3. Continue to the next
     * waypoint as normal.
     *
     * @param intersection Intersection to approach.
     * @param robotPos     Robot's current position/rotation.
     * @return Final motor speeds.
     */
    private double[] handlePointTurnIntersection(TaggedIntersection intersection, Pose2d robotPos) {
        /*
         * Point turn intersections are handled very differently than general intersections. Instead of "curving" around
         * the point, the robot will decelerate and perform a point turn.
         */
        PointTurnWaypoint waypoint = (PointTurnWaypoint) intersection.taggedPoint;
        double targetAngle;
        if (!waypoint.hasTraversed() && PathfindingUtil.positionEqualsWithBuffer(robotPos.getTranslation(),
                waypoint.getTranslation(), waypoint.getPositionBuffer())) {
            GeneralWaypoint next = (GeneralWaypoint) get(intersection.waypointIndex + 1);
            if (next.usingPreferredAngle()) {
                if (PathfindingUtil.rotationEqualsWithBuffer(robotPos.getHeading(), next.getPreferredAngle(),
                        waypoint.getRotationBuffer())) {
                    // If the robot has reached the point and is at the preferredAngle, then the point is traversed.
                    waypoint.setTraversed();
                }
                // set target angle
                targetAngle = next.getPreferredAngle();
            } else {
                targetAngle = Math.atan2(next.getY() - robotPos.getY(), next.getX() - robotPos.getX());

                if (PathfindingUtil.rotationEqualsWithBuffer(robotPos.getHeading(), targetAngle,
                        waypoint.getRotationBuffer())) {
                    // If the robot has reached the point and is at the target angle, then the point is traversed.
                    waypoint.setTraversed();
                }
            }
            // turn to next target angle
            return PathfindingUtil.moveToPosition(robotPos,
                    new Pose2d(intersection.intersection, new Rotation2d(targetAngle)), true);
        } else {
            // degenerates to a general intersection
            return handleGeneralIntersection(intersection, robotPos);
        }
    }

    /**
     * Returns the motor speeds required to approach the given interrupt intersection. This will cause the robot to
     * behave as follows: 1. Approach and decelerate to the waypoint. 2. Perform a point turn / align with the preferred
     * angle. 3. Perform the interrupt action. 4. Continue to the next waypoint as normal.
     *
     * @param intersection Intersection to approach.
     * @param robotPos     Robot's current position/rotation.
     * @return Final motor speeds.
     */
    private double[] handleInterruptIntersection(TaggedIntersection intersection, Pose2d robotPos) {
        /*
         * Interrupt intersections are handled similarly to point turn intersections. Instead of continuing directly
         * after it has turned, the robot will stop and perform the interrupt actions.
         */
        InterruptWaypoint waypoint = (InterruptWaypoint) intersection.taggedPoint;
        double targetAngle;

        if (!waypoint.hasTraversed() && PathfindingUtil.positionEqualsWithBuffer(robotPos.getTranslation(),
                waypoint.getTranslation(), waypoint.getPositionBuffer())) {
            GeneralWaypoint next = (GeneralWaypoint) get(intersection.waypointIndex + 1);
            if (waypoint.getType() == WaypointType.END) {
                if (waypoint.usingPreferredAngle() && !PathfindingUtil.rotationEqualsWithBuffer(robotPos.getHeading(),
                        waypoint.getPreferredAngle(), waypoint.getRotationBuffer())) {
                    targetAngle = waypoint.getPreferredAngle();
                } else {
                    waypoint.setTraversed();
                    return STOP;
                }
            } else {
                // use preferred angle if available
                targetAngle = next.usingPreferredAngle() ? next.getPreferredAngle() :
                              Math.atan2(next.getY() - robotPos.getY(), next.getX() - robotPos.getX());

                if (PathfindingUtil.rotationEqualsWithBuffer(robotPos.getHeading(), targetAngle,
                        waypoint.getRotationBuffer())) {
                    // If the robot has reached the point and is at the preferredAngle, then the point is traversed.
                    waypoint.setTraversed();
                    // queue action
                    interruptActionQueue.add(waypoint);
                    // robot doesn't move while performing action
                    return STOP;
                }
            }
            // rotate to target angle
            return PathfindingUtil.moveToPosition(robotPos,
                    new Pose2d(intersection.intersection, new Rotation2d(targetAngle)), true);
        } else {
            // use preferred angle if available
            targetAngle = waypoint.usingPreferredAngle() ? waypoint.getPreferredAngle() :
                          Math.atan2(intersection.intersection.getY() - robotPos.getY(),
                                  intersection.intersection.getX() - robotPos.getX());
            // move to intersection
            return PathfindingUtil.moveToPosition(robotPos,
                    new Pose2d(intersection.intersection, new Rotation2d(targetAngle)), false);
        }
    }

    /**
     * Returns the motor speeds required to approach the given end intersection. This will cause the robot to behave as
     * follows: 1. Approach and decelerate to the end point. 2. Turn to face the preferred angle (if provided). 3. Mark
     * the path as complete.
     *
     * @param intersection Intersection to approach.
     * @param robotPos     Robot's current position/rotation.
     * @return Final motor speeds.
     */
    private double[] handleEndIntersection(TaggedIntersection intersection, Pose2d robotPos) {
        /*
         * End intersections are handled the same way as interrupt intersections.
         */
        return handleInterruptIntersection(intersection, robotPos);
    }

    /**
     * Resets all the waypoints/timeouts/actions in this path. Called by {@link #init()}.
     */
    public void reset() {
        resetTimeouts();
        // reset waypoints
        for (Waypoint waypoint : this) {
            if (waypoint instanceof GeneralWaypoint) {
                ((GeneralWaypoint) waypoint).reset();
            }
        }
        triggeredActions.forEach(TriggeredAction::reset);
    }

    /**
     * Resets all timeouts.
     *
     * @return This path, used for chaining methods.
     */
    public Path resetTimeouts() {
        timedOut = false;
        lastWaypointTimestamp = System.currentTimeMillis();
        return this;
    }

    /**
     * Checks to make sure the path is legal. If not, an exception is thrown.
     * <p>
     * In order for a path to be considered legal it must have: - At least 2 waypoints. - Begin with a StartWaypoint -
     * End with an EndWaypoint - Not contain any other StartWaypoints or EndWaypoints
     *
     * @throws IllegalStateException If the path is not legal.
     */
    public void verify() {
        if (size() < 2) {
            throw new IllegalStateException("Path must have at least two waypoints.");
        }
        if (get(0).getType() != WaypointType.START) {
            throw new IllegalStateException("Path must start with a StartWaypoint.");
        }
        if (get(size() - 1).getType() != WaypointType.END) {
            throw new IllegalStateException("Path must end with an EndWaypoint.");
        }
        boolean contains = subList(1, size() - 1)
                .stream()
                .anyMatch(
                        waypoint -> waypoint.getType() == WaypointType.START
                                || waypoint.getType() == WaypointType.END
                );
        if (contains) {
            throw new IllegalStateException(
                    "Path must not have end and start waypoints anywhere other than the first and"
                            + " last spot.");
        }
    }

    private PathMotionProfile getDefaultMotionProfile() {
        if (defaultMotionProfile != null) {
            return defaultMotionProfile;
        }
        return new PathMotionProfile() {
            @Override
            public void decelerate(double[] motorSpeeds, double distanceToTarget, double speed,
                    double configuredMovementSpeed, double configuredTurnSpeed) {
                if (distanceToTarget < 0.15) {
                    motorSpeeds[0] *= configuredMovementSpeed * ((distanceToTarget * 10) + 0.1);
                    motorSpeeds[1] *= configuredMovementSpeed * ((distanceToTarget * 10) + 0.1);
                } else {
                    motorSpeeds[0] *= configuredMovementSpeed;
                    motorSpeeds[1] *= configuredMovementSpeed;
                }
                motorSpeeds[2] *= configuredTurnSpeed;
            }

            @Override
            public void accelerate(double[] motorSpeeds, double distanceFromTarget, double speed,
                    double configuredMovementSpeed, double configuredTurnSpeed) {
                if (distanceFromTarget < 0.15) {
                    motorSpeeds[0] *= configuredMovementSpeed * ((distanceFromTarget * 10) + 0.1);
                    motorSpeeds[1] *= configuredMovementSpeed * ((distanceFromTarget * 10) + 0.1);
                } else {
                    motorSpeeds[0] *= configuredMovementSpeed;
                    motorSpeeds[1] *= configuredMovementSpeed;
                }
                motorSpeeds[2] *= configuredTurnSpeed;
            }
        };
    }

    /**
     * Sets the default motion profile.
     *
     * @param profile Motion profile to be set.
     */
    public static void setDefaultMotionProfile(PathMotionProfile profile) {
        defaultMotionProfile = profile;
    }

    /**
     * Sets a timeout for the entire path. If the path does not complete within the timeout period, it will abort.
     *
     * @param timeoutMilliseconds Timeout to be set.
     * @return This path, used for chaining methods.
     */
    public Path setPathTimeout(long timeoutMilliseconds) {
        this.timeoutMilliseconds = timeoutMilliseconds;
        return this;
    }

    /**
     * Sets the path type to the specified type. By default the path type is WAYPOINT_ORDERING_CONTROLLED.
     *
     * @param pathType Path type to be set.
     * @return This path, used for chaining methods.
     */
    public Path setPathType(PathType pathType) {
        this.pathType = pathType;
        return this;
    }

    /**
     * Sets this path's motion profile to the provided PathMotionProfile.
     *
     * @param profile PathMotionProfile to be set.
     * @return This path, used for chaining methods.
     * @throws NullPointerException If the controller is null.
     */
    public Path setMotionProfile(PathMotionProfile profile) {
        if (profile == null) {
            throw new NullPointerException("The motion profile connot be null");
        }
        motionProfile = profile;
        return this;
    }

    /**
     * Sets the timeouts of n waypoints where n is the amount of arguments provided. The nth waypoint timeout is set the
     * the nth argument given.
     *
     * @param timeouts Timeouts to be set.
     * @return This path, used for chaining methods.
     */
    public Path setWaypointTimeouts(long... timeouts) {
        for (int i = 0; i < size() && i < timeouts.length; i++) {
            if (get(i) instanceof GeneralWaypoint) {
                ((GeneralWaypoint) get(i)).setTimeout(timeouts[i]);
            }
        }
        return this;
    }

    /**
     * Sets the timeout for each individual waypoint to be the value provided. This is not recommended.
     *
     * @param timeout Universal timeout to be set.
     * @return This path, used for chaining methods.
     */
    public Path setWaypointTimeouts(long timeout) {
        for (Waypoint waypoint : this) {
            if (waypoint instanceof GeneralWaypoint) {
                ((GeneralWaypoint) waypoint).setTimeout(timeout);
            }
        }
        return this;
    }

    /**
     * Configures the retrace settings. The default values are as follows: movementSpeed = 1 turnSpeed = 1
     *
     * @param movementSpeed Movement speed to be set.
     * @param turnSpeed     Turn speed to be set.
     * @return This path, used for chaining methods.
     */
    public Path setRetraceSettings(double movementSpeed, double turnSpeed) {
        retraceMovementSpeed = normalizeSpeed(movementSpeed);
        retraceTurnSpeed = normalizeSpeed(turnSpeed);
        return this;
    }

    /**
     * Enables retrace. If the robot loses the path and this is enabled, the robot will retrace its moves to try to re
     * find the path. This is enabled by default.
     *
     * @return This path, used for chaining methods.
     */
    public Path enableRetrace() {
        retraceEnabled = true;
        return this;
    }

    /**
     * Disables retrace.
     *
     * @return This path, used for chaining methods.
     */
    public Path disableRetrace() {
        retraceEnabled = false;
        return this;
    }

    /**
     * Adds the provided TriggeredActions to the path. These are handled automatically.
     *
     * @param actions TriggeredActions to be added.
     * @return This path, used for chaining methods.
     */
    public Path addTriggeredActions(TriggeredAction... actions) {
        triggeredActions.addAll(Arrays.asList(actions));
        return this;
    }

    /**
     * Removes the first instance of the provided TriggeredAction from the path.
     *
     * @param action TriggeredAction to be removed
     * @return This path, used for chaining methods.
     */
    public Path removeTriggeredAction(TriggeredAction action) {
        triggeredActions.remove(action);
        return this;
    }

    /**
     * Removes all TriggeredActions from the path.
     *
     * @return This path, used for chaining methods.
     */
    public Path clearTriggeredActions() {
        triggeredActions.clear();
        return this;
    }

    /**
     * Checks if the path is legal. See {@link #verify()}.
     *
     * @return true if this path is legal, false otherwise.
     */
    public boolean isLegalPath() {
        try {
            verify();
            initComplete = false;
        } catch (IllegalStateException e) {
            return false;
        }
        return true;
    }

    /**
     * @return true if the path has been completed, false otherwise.
     */
    public boolean isFinished() {
        if (size() > 0 && get(size() - 1).getType() == WaypointType.END) {
            return ((EndWaypoint) get(size() - 1)).isFinished();
        }
        return false;
    }

    /**
     * @return Whether the path has timed out.
     */
    public boolean timedOut() {
        return timedOut;
    }

    /**
     * Calls the loop() method on all TriggeredActions in this path.
     */
    private void loopTriggeredActions() {
        triggeredActions.forEach(TriggeredAction::loop);
    }

    /**
     * Performs all queued interrupt actions.
     */
    private void runQueuedInterruptActions() {
        while (!interruptActionQueue.isEmpty()) {
            interruptActionQueue.remove().performAction();
        }
    }

    /**
     * Adjusts the motor speeds based on this path's motion profile.
     *
     * @param speeds       Speeds to be adjusted.
     * @param intersection The tagged intersection.
     * @param robotPos     Position of the robot.
     */
    private void adjustSpeedWithProfile(double[] speeds, TaggedIntersection intersection,
            Translation2d robotPos) {
        Translation2d awayPoint = null;
        for (int i = intersection.waypointIndex - 1; i >= 0; i--) {
            Waypoint temp = get(i);
            if (temp.getType() == WaypointType.START || temp instanceof PointTurnWaypoint) {
                awayPoint = temp.getPose().getTranslation();
                break;
            }
        }
        // paranoia, should never happen
        if (awayPoint == null) {
            throw new IllegalStateException("Path has broken.");
        }
        Translation2d toPoint = intersection.taggedPoint.getPose().getTranslation();

        double distanceTo = robotPos.getDistance(toPoint);
        double distanceAway = robotPos.getDistance(awayPoint);

        if (distanceAway < distanceTo) {
            // closer to away point, we are accelerating
            motionProfile.processAccelerate(speeds, distanceAway,
                    ((GeneralWaypoint) intersection.taggedPoint).getMovementSpeed(),
                    ((GeneralWaypoint) intersection.taggedPoint).getTurnSpeed());
        } else {
            // closer to destination, we are decelerating
            motionProfile.processDecelerate(speeds, distanceTo,
                    ((GeneralWaypoint) intersection.taggedPoint).getMovementSpeed(),
                    ((GeneralWaypoint) intersection.taggedPoint).getTurnSpeed());
        }
    }

    /**
     * Normalizes the given raw speed to be in the range [0, 1]
     *
     * @param speed Raw speed value to be normalized.
     * @return Normalized speed value.
     */
    protected double normalizeSpeed(double speed) {
        return MathUtils.clamp(speed, 0, 1);
    }

    /**
     * Normalizes motor powers, scaling the strafe and drive powers and clipping the turn power.
     *
     * @param speeds Array of motor powers, with the elements being strafe, drive, and turn power, respectively.
     */
    private void normalizeMotorSpeeds(double[] speeds) {
        double max = Math.max(Math.abs(speeds[0]), Math.abs(speeds[1]));

        if (max > 1) {
            speeds[0] /= max;
            speeds[1] /= max;
        }

        speeds[2] = MathUtils.clamp(speeds[2], -1, 1);
    }

    /**
     * This private class is used to store additional information associated with an intersection.
     */
    private static class TaggedIntersection {
        // location of the intersection
        public Translation2d intersection;

        // waypoint associated with this intersection
        public Waypoint taggedPoint;

        // The associated waypoint's index in the path.
        public int waypointIndex;

        /**
         * Constructs a TaggedIntersection object with the given values.
         *
         * @param intersection  Location of the intersection.
         * @param taggedPoint   Waypoint associated with the intersection.
         * @param waypointIndex Index of the associated waypoint in the path.
         */
        public TaggedIntersection(Translation2d intersection, Waypoint taggedPoint, int waypointIndex) {
            this.intersection = intersection;
            this.taggedPoint = taggedPoint;
            this.waypointIndex = waypointIndex;
        }
    }
}