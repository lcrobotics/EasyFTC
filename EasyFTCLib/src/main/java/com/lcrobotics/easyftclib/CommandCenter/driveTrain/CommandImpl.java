/**
 * WARNING: THIS CODE WAS WRITTEN BY TWO IDIOTS, PROBABLY WHILE SOME FORM OF HIGH. USE AT YOUR OWN
 * RISK.
 */

package com.lcrobotics.easyftclib.CommandCenter.driveTrain;

import android.support.annotation.NonNull;

import com.qualcomm.robotcore.hardware.IntegratingGyroscope;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * {@link CommandImpl} is the base implementation of a command. This class should never be used by
 * the user, but rather extend this class when creating their own Commands from scratch.
 */
public class CommandImpl implements Command {

    public DistanceUnit distanceUnit;
    public AngleUnit angleUnit;
    public IntegratingGyroscope gyroscope;

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
     * gives SmartTrain motor powers to set
     * @param motorPowers array of powers that the motors should be set to
     */
    @Override
    public void setMotorPowers(double[] motorPowers) {
        CommandData.motorPowers = motorPowers;
    }

    /**
     *
     * @param parameters an instance of the {@link Parameters} class, defines how this command
     *                   operates
     *
     * @return whether the initialization was successful, however, there should be no reason for
     * it to ever fail
     */
    @Override
    public boolean initialize(@NonNull Parameters parameters) {
        this.distanceUnit = parameters.distanceUnit;
        this.angleUnit = parameters.angleUnit;
        this.gyroscope = parameters.gyroscope;
        return true;
    }
}
