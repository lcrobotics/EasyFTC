package com.lcrobotics.easyftclib.pathfinding;

public abstract class PathMotionProfile {
    // keep track of method calls
    private double lastDistanceToTarget;
    private long lastCallTimeStamp;

    // true if last call was deceleration
    private boolean lastCallType;

    /**
     * Default constructor for a PathMotionProfile
     */
    public PathMotionProfile() {
        lastDistanceToTarget = lastCallTimeStamp = -1;
        lastCallType = true;
    }


    /**
     * Adjusts the motor speeds to decelerate the robot as it approaches a target based on this motion profile.
     * Called by Path as a pipeline to decelerate().
     *
     * @param motorSpeeds             Raw motor speeds.
     * @param distanceToTarget        Distance the robot is from it's target.
     * @param configuredMovementSpeed Configured movement speed.
     * @param configuredTurnSpeed     Configured turn speed.
     */
    public void processDecelerate(double[] motorSpeeds, double distanceToTarget, double configuredMovementSpeed, double configuredTurnSpeed) {
        if (lastCallType) {
            // Call decelerate().
            decelerate(motorSpeeds, distanceToTarget,
                    (lastDistanceToTarget - distanceToTarget) / ((System.nanoTime() - lastCallTimeStamp) * 1e9),
                    configuredMovementSpeed, configuredTurnSpeed);
        } else {
            // If the last call was not a decelerate, then skip the first call.
            lastCallType = true;
        }
        // Update fields.
        lastDistanceToTarget = distanceToTarget;
        lastCallTimeStamp = System.nanoTime();
    }

    /**
     * Adjusts the motor speeds to accelerate the robot as it approaches a target based on this motion profile.
     * Called by Path as a pipeline to accelerate().
     *
     * @param motorSpeeds             Raw motor speeds.
     * @param distanceFromTarget      Distance the robot is from it's target.
     * @param configuredMovementSpeed Configured movement speed.
     * @param configuredTurnSpeed     Configured turn speed.
     */
    public void processAccelerate(double[] motorSpeeds, double distanceFromTarget, double configuredMovementSpeed, double configuredTurnSpeed) {
        if (!lastCallType) {
            // Call accelerate().
            accelerate(motorSpeeds, distanceFromTarget,
                    (distanceFromTarget - lastDistanceToTarget) / ((System.nanoTime() - lastCallTimeStamp) * 1e9),
                    configuredMovementSpeed, configuredTurnSpeed);
        } else {
            // If the last call was not a decelerate, then skip the first call.
            lastCallType = false;
        }
        // Update fields.
        lastDistanceToTarget = distanceFromTarget;
        lastCallTimeStamp = System.nanoTime();
    }

    /**
     * Decelerates the motor speeds. Used as the robot approaches a target
     *
     * @param motorSpeeds             Raw motor speeds.
     * @param distanceToTarget        Distance the robot is from the target destination.
     * @param speed                   The robot's calculated speed, in units/second.
     * @param configuredMovementSpeed Configured movement speed.
     * @param configuredTurnSpeed     Configured turn speed.
     */
    public abstract void decelerate(double[] motorSpeeds, double distanceToTarget, double speed,
                                    double configuredMovementSpeed, double configuredTurnSpeed);

    /**
     * Accelerates the motor speeds. Used as the robot approaches a target.
     *
     * @param motorSpeeds             Raw motor speeds.
     * @param distanceFromTarget      Distance the robot is from the target.
     * @param speed                   The robot's calculated speed, in units/second.
     * @param configuredMovementSpeed Configured movement speed.
     * @param configuredTurnSpeed     Configured turn speed.
     */
    public abstract void accelerate(double[] motorSpeeds, double distanceFromTarget, double speed,
                                    double configuredMovementSpeed, double configuredTurnSpeed);
}
