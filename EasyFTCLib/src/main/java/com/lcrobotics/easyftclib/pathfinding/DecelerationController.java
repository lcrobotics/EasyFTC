package com.lcrobotics.easyftclib.pathfinding;

public abstract class DecelerationController {
    // keep track of method calls
    private double lastDistanceToTarget;
    private long lastCallTimeStamp;

    public DecelerationController() {
        lastDistanceToTarget = lastCallTimeStamp = -1;
    }

    /**
     * Adjusts the motor speeds as the robot approaches a target point. Called by Path while decelerating.
     * Basically just a pipeline to {@link }
     *
     * @param motorSpeeds             Raw motor speeds.
     * @param distanceToTarget        Distance from the robot to the target.
     * @param configuredMovementSpeed Configured movement speed of the target.
     * @param configuredTurnSpeed     Configured turn speed of the target.
     */
    public void process(double[] motorSpeeds, double distanceToTarget,
                        double configuredMovementSpeed, double configuredTurnSpeed) {

        decelerateMotorSpeeds(motorSpeeds, distanceToTarget, lastDistanceToTarget,
                System.nanoTime() - lastCallTimeStamp,
                configuredMovementSpeed, configuredTurnSpeed);

        lastDistanceToTarget = distanceToTarget;
        lastCallTimeStamp = System.nanoTime();
    }

    /**
     * User-defined
     *
     * @param motorSpeeds             Raw motor speeds.
     * @param distanceToTarget        Distance the robot is from the target destination.
     * @param lastDistanceToTarget    Previous distance to target, can be used to estimate speed. This value is -1 if this is the first time this method is called.
     * @param timeSinceLastCallNano   Time since the last time with method was called (in nanoseconds). This value is -1 if this is the first time this method is called.
     * @param configuredMovementSpeed Configured movement speed.
     * @param configuredTurnSpeed     Configured turn speed.
     */
    public abstract void decelerateMotorSpeeds(double[] motorSpeeds, double distanceToTarget,
                                               double lastDistanceToTarget, double timeSinceLastCallNano,
                                               double configuredMovementSpeed, double configuredTurnSpeed);
}
