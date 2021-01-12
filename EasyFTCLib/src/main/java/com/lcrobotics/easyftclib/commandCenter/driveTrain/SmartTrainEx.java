
package com.lcrobotics.easyftclib.commandCenter.driveTrain;

import android.support.annotation.NonNull;

import com.lcrobotics.easyftclib.commandCenter.hardware.MotorEx;

import java.util.Arrays;

/**
 * Bingo Bango Bongo, this code is shit
 */
public class SmartTrainEx {
    private final double radius;
    private final MotorEx[] motors;

    /**
     * Base constructor for SmartTrain
     * @param wheelRadius radius of drive wheels
     * @param motors array of motors
     */
    public SmartTrainEx(double wheelRadius, MotorEx... motors) {
        radius = wheelRadius;
        this.motors = motors;
    }

    public void setPower(double power) {
        for (MotorEx motor : motors) {
            motor.set(power);
        }
    }

    public void setPower(double... powers) {
        if (powers.length != motors.length) {
            throw new RuntimeException("Invalid number of powers for amount of motors");
        }
        for (int i = 0; i < powers.length; i++) {
            motors[i].set(powers[i]);
        }
    }

    public void setPositionCoefficient(double p) {
        for (MotorEx motor : motors) {
            motor.setPositionCoefficient(p);
        }
    }

    public void setVelocityCoefficients(double p, double i, double d) {
        for (MotorEx motor : motors) {
            motor.setVelocityCoefficients(p, i, d);
        }
    }

    public void setFeedforwardCoefficients(double s, double v) {
        for (MotorEx motor : motors) {
            motor.setFeedforwardCoefficients(s, v);
        }
    }

    public MotorEx[] getMotors() {
        return motors;
    }

    @NonNull
    @Override
    public String toString() {
        return "SmartTrainEx{" +
                "motors=" + Arrays.toString(motors) +
                ", radius=" + radius +
                '}';
    }
}