
package com.lcrobotics.easyftclib.commandCenter.driveTrain;

import com.lcrobotics.easyftclib.commandCenter.hardware.Motor;

/**
 * Bingo Bango Bongo, this code is shit
 */
public class SmartTrainEx {
    private Motor[] motors;
    private final double radius;

    /**
     * Base constructor for SmartTrain
     * @param wheelRadius radius of drive wheels
     * @param motors array of motors
     */
    public SmartTrainEx(double wheelRadius, Motor... motors) {
        radius = wheelRadius;
        this.motors = motors;
    }

    public void setPower(double power) {
        for (Motor motor : motors) {
            motor.set(power);
        }
    }

    public void setPower(double... powers) {
        if (powers.length != motors.length) {
            throw new RuntimeException("invalid number of powers for amount of motors");
        }
        for (int i = 0; i < powers.length; i++) {
            motors[i].set(powers[i]);
        }
    }
}