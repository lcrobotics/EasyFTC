package com.lcrobotics.easyftclib.tools.kinematics;

import android.support.annotation.NonNull;

import java.util.Locale;
import java.util.stream.DoubleStream;

public class MecanumDriveWheelSpeeds {
    /**
     * Speed of the front left wheel.
     */
    public double frontLeftSpeed;

    /**
     * Speed of the front right wheel.
     */
    public double frontRightSpeed;

    /**
     * Speed of the back left wheel.
     */
    public double backLeftSpeed;

    /**
     * Speed of the back right wheel.
     */
    public double backRightSpeed;

    /**
     * Constructs a MecanumDriveWheelSpeeds with zeros for all speeds.
     */
    public MecanumDriveWheelSpeeds() {
    }

    /**
     * Constructs a MecanumDriveWheelSpeeds with given speeds.
     *
     * @param frontLeftSpeed  The speed of the front-left wheel.
     * @param frontRightSpeed The speed of the front-right wheel.
     * @param backLeftSpeed   The speed of the back-left wheel.
     * @param backRightSpeed  The speed of the back-right wheel.
     */
    public MecanumDriveWheelSpeeds(double frontLeftSpeed, double frontRightSpeed,
                                   double backLeftSpeed, double backRightSpeed) {
        this.frontLeftSpeed = frontLeftSpeed;
        this.frontRightSpeed = frontRightSpeed;
        this.backLeftSpeed = backLeftSpeed;
        this.backRightSpeed = backRightSpeed;
    }

    /**
     * Normalizes the wheel speeds using some max attainable speed. Sometimes, after inverse
     * kinematics, the speed(s) of one or more wheels may be above their max attainable speed.
     * To fix this, we "normalize" the wheel speeds to make sure all wheel speeds are below that
     * absolute maximum while maintaining the ratio between wheel speeds.
     *
     * @param attainableMaxSpeed The absolute max speed that a wheel can attain.
     */
    public void normalize(double attainableMaxSpeed) {
        // get max speed
        double realMaxSpeed = DoubleStream
                .of(frontLeftSpeed, frontRightSpeed, backLeftSpeed, backRightSpeed)
                .max()
                .getAsDouble();

        if (realMaxSpeed > attainableMaxSpeed) {
            frontLeftSpeed = frontLeftSpeed / realMaxSpeed * attainableMaxSpeed;
            frontRightSpeed = frontRightSpeed / realMaxSpeed * attainableMaxSpeed;
            backLeftSpeed = backLeftSpeed / realMaxSpeed * attainableMaxSpeed;
            backRightSpeed = backRightSpeed / realMaxSpeed * attainableMaxSpeed;
        }
    }

    @NonNull
    @Override
    public String toString() {
        return String.format(Locale.US,
                "MecanumDriveWheelSpeeds(Front Left: %.2f m/s, Front Right: %.2f m/s, "
                        + "Back Left: %.2f m/s, Back Right: %.2f m/s)",
                frontLeftSpeed,
                frontRightSpeed,
                backLeftSpeed,
                backRightSpeed);
    }
}
