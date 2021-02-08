package com.lcrobotics.easyftclib.tools.kinematics;

import android.support.annotation.NonNull;

import com.lcrobotics.easyftclib.tools.geometry.Rotation2d;

import java.util.Locale;

public class ChassisSpeeds {
    /**
     * Represents forward velocity in robot frame of reference. (Fwd is +)
     */
    public double velocityX;
    /**
     * Represents sideways velocity in robot frame of reference. (Left is +)
     */
    public double velocityY;
    /**
     * Represents the angular velocity of the robot frame. (CCW is +)
     */
    public double omegaRadiansPerSecond;

    public ChassisSpeeds() {
    }

    /**
     * Constructs a ChassisSpeeds object.
     *
     * @param velocityX             Forward velocity.
     * @param velocityY             Sideways velocity.
     * @param omegaRadiansPerSecond Angular velocity.
     */
    public ChassisSpeeds(double velocityX, double velocityY, double omegaRadiansPerSecond) {
        this.velocityX = velocityX;
        this.velocityY = velocityY;
        this.omegaRadiansPerSecond = omegaRadiansPerSecond;
    }

    /**
     * Converts a user provided field-relative set of speeds into a robot-relative ChassisSpeeds
     * object.
     *
     * @param velocityX             The component of speed in the x direction relative to the field.
     *                              Positive x is away from your alliance wall.
     * @param velocityY             The component of speed in the y direction relative to the field.
     *                              Positive y is to your left when standing behind the alliance wall.
     * @param omegaRadiansPerSecond The angular rate of the robot.
     * @param robotAngle            The angle of the robot as measured by a gyroscope. The robot's angle is
     *                              considered to be zero when it is facing directly away from your alliance station wall.
     *                              Remember that this should be CCW positive.
     * @return ChassisSpeeds object representing the speeds in the robot's frame of reference.
     */
    public static ChassisSpeeds fromFieldRelativeSpeeds(
            double velocityX, double velocityY,
            double omegaRadiansPerSecond, Rotation2d robotAngle) {
        return new ChassisSpeeds(
                velocityX * robotAngle.getCos() + velocityY * robotAngle.getSin(),
                -velocityX * robotAngle.getSin() + velocityY * robotAngle.getCos(),
                omegaRadiansPerSecond);
    }

    @NonNull
    @Override
    public String toString() {
        return String.format(Locale.US,
                "ChassisSpeeds(Vx: %.2f m/s, Vy: %.2f m/s, Omega: %.2f rad/s)",
                velocityX, velocityY, omegaRadiansPerSecond);
    }
}
