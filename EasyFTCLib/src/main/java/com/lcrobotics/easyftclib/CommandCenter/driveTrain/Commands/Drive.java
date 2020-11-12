package com.lcrobotics.easyftclib.CommandCenter.driveTrain.Commands;

import android.support.annotation.NonNull;

import com.lcrobotics.easyftclib.CommandCenter.driveTrain.CommandData;
import com.lcrobotics.easyftclib.CommandCenter.driveTrain.Commands.CommandImpl;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

import java.util.Arrays;

/**
 * This command is designed to drive on a given angle for a certain distance.
 * It uses the gyroscope provided to correct any drift along the z axis.
 */
public class Drive extends CommandImpl {
    public double distance;
    public double angle;
    public double power;
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
        CommandData.moveCounts = new int[CommandData.motorCount];
        // fill array
        Arrays.fill(CommandData.moveCounts, moveCounts);
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

        double leftSpeed = power - steer;
        double rightSpeed = power + steer;
        // normalize speeds if either speed goes above 1
        double max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));

        if (max > 1.0) {
            leftSpeed /= max;
            rightSpeed /= max;
        }
        // set motor powers

        setMotorPowers(motorPowers);
        return 1;
    }

    @Override
    public boolean initialize(@NonNull Parameters parameters) {
        return super.initialize(parameters);
    }
}
