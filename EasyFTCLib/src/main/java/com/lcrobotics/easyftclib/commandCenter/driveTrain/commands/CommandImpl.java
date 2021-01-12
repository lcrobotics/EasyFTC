/**
 * WARNING: THIS CODE WAS WRITTEN BY TWO IDIOTS, PROBABLY WHILE SOME FORM OF HIGH. USE AT YOUR OWN
 * RISK.
 */

package com.lcrobotics.easyftclib.commandCenter.driveTrain.commands;

import android.support.annotation.NonNull;

import com.lcrobotics.easyftclib.commandCenter.old.CommandData;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * {@link CommandImpl} is the base implementation of a command. This class should never be used by
 * the user, but rather extend this class when creating their own Commands from scratch.
 */
public class CommandImpl implements Command {

    public DistanceUnit distanceUnit;
    public AngleUnit angleUnit;
    public IntegratingGyroscope gyroscope;

    @Override
    public int init() {
        CommandData.needsEncoders = false;
        return 0;
    }

    /**
     * this method defines how the command goes about its path. This will be specific
     * to each command, so any user defined commands should write their own.
     * @return an integer that represents the outcome of updating this command (can either be
     * an error code or 0 for success).
     */
    @Override
    public int update() {
        return 0;
    }

    /**
     * Sets motor powers for each side of the robot
     * @param leftPower Power for the left wheels
     * @param rightPower Power for the right wheels
     */
    public void setMotorPowers(double leftPower, double rightPower) {
        double[] motorPowers = new double[CommandData.motorCount];
        // alternate between left and right
        for (int i = 0; i < motorPowers.length; i++) {
            if (i % 2 == 0) {
                motorPowers[i] = leftPower;
            } else {
                motorPowers[i] = rightPower;
            }
        }

        CommandData.motorPowers = motorPowers;
    }
    /**
     * Gives SmartTrain motor powers to set
     * @param motorPowers array of powers that the motors should be set to
     */
    @Override
    public void setMotorPowers(double[] motorPowers) {
        CommandData.motorPowers = motorPowers;
    }

    /**
     * Initialize Command with given parameters
     * @param parameters an instance of the {@link Parameters} class, defines how this command
     *                   operates
     *
     * @return Whether the initialization was successful, however, there should be no reason for
     * it to ever fail
     */
    @Override
    public boolean initialize(@NonNull Parameters parameters) {
        this.distanceUnit = parameters.distanceUnit;
        this.angleUnit = parameters.angleUnit;
        this.gyroscope = parameters.gyroscope;
        return true;
    }

    /**
     * Initialize Command with default parameters
     * @return Whether the initialization was successful, however, there should be no reason for
     * it to ever fail
     */
    @Override
    public boolean initialize() {
        return initialize(new Parameters());
    }

    /**
     * Returns the error between the target angle and the robot's current heading
     * @param currAngle Current angle read from gyroscope
     * @param targetAngle Desired angle (relative to last Gyro reset).
     * @return Error angle: Degrees in the range +/- 180.
     * +ve error means the robot should turn LEFT (CCW) to reduce error.
     */
    public double getError(double targetAngle, double currAngle) {

        double robotError;
        // calculate error in -179 to +180 range
        // make sure angle is in correct range
        robotError = targetAngle - currAngle;
        while (robotError > 180) robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }
    /**
     * Returns desired steering force.  +/- 1 range.  +ve = steer left
     *
     * @param error  Error angle in robot relative degrees
     * @param PCoeff Proportional Gain Coefficient
     * @return Desired steering force
     */
    public double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }

    /**
     * Returns current heading of robot on the z-axis
     * @return Current heading
     */
    public double getZHeading() {
        if (gyroscope == null) {
            return CommandData.angularOrientation.firstAngle;
        } else {
            return gyroscope.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, angleUnit).firstAngle;
        }
    }


}
