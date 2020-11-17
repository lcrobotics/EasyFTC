package com.lcrobotics.easyftclib.CommandCenter.driveTrain.Commands;

import com.lcrobotics.easyftclib.CommandCenter.driveTrain.CommandData;

/**
 * This command is designed to drive on a given angle for a certain distance.
 * It uses the gyroscope provided to correct any drift along the z axis.
 */
public class Drive extends CommandImpl {
    public double distance;
    public double angle;
    public double power;
    private int[] targetPositions;
    public static final double P_DRIVE_COEFF = 0.15;
    /**
     * @param distance distance to drive
     * @param angle angle to drive on relative to
     */
    public Drive(double distance, double angle, double power) {
        this.distance = distance;
        this.angle = angle;
        this.power = power;
    }

    @Override
    public int init() {
        // calculate encoder ticks needed to move distance provided
        int moveCounts = (int) (distanceUnit.toCm(distance) * CommandData.countsPerCM);
        // set target positions
        targetPositions = new int[CommandData.motorCount];
        for (int i = 0; i < targetPositions.length; i++) {
            targetPositions[i] = CommandData.currentPositions[i] + moveCounts;
        }
        CommandData.targetPositions = targetPositions;
        CommandData.needsEncoders = true;
        return 0;
    }

    @Override
    public int update() {
        // heading along z axis read from gyroscope
        double zAngle = getZHeading();
        // get error relative to desired heading
        double error = getError(angle, zAngle);
        double steer = getSteer(error, P_DRIVE_COEFF);
        // if command is going backwards, reverse steer
        if (distance < 0) {
            steer *= -1;
        }

        double leftPower = power - steer;
        double rightPower = power + steer;
        // normalize powers if either power goes above 1
        double max = Math.max(Math.abs(leftPower), Math.abs(rightPower));

        if (max > 1.0) {
            leftPower /= max;
            rightPower /= max;
        }
        // set motor powers
        setMotorPowers(leftPower, rightPower);
        return isBusy()? 1 : 0;
    }

    /**
     * checks if any motor is still reaching a target position
     * @return true if any motor is busy, otherwise false
     */
    public boolean isBusy() {
        for (int i = 0; i < CommandData.motorCount; i++) {
            if (Math.abs(CommandData.currentPositions[i] - targetPositions[i]) > 7) {
                return true;
            }
        }
        return false;
    }
}
