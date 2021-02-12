package com.lcrobotics.easyftclib.tools.geometry;

import android.support.annotation.NonNull;
import android.support.annotation.Nullable;

import java.util.Locale;
import java.util.Objects;

/**
 * A change in distance along an arc since last position update.
 */
public class Twist2d {
    /**
     * Linear "dx" component
     */
    public double dx;

    /**
     * Linear "dy" component
     */
    public double dy;

    /**
     * Angular "dtheta" component (radians).
     */
    public double dtheta;

    public Twist2d() {
    }

    /**
     * Constructs a Twist2d with the given values.
     *
     * @param dx     Change in x direction relative to robot.
     * @param dy     Change in y direction relative to robot.
     * @param dtheta Change in angle relative to robot.
     */
    public Twist2d(double dx, double dy, double dtheta) {
        this.dx = dx;
        this.dy = dy;
        this.dtheta = dtheta;
    }

    /**
     * Checks equality between this Twist2d and another object.
     *
     * @param obj The other object.
     * @return Whether the two objects are equal.
     */
    @Override
    public boolean equals(@Nullable Object obj) {
        if (obj instanceof Twist2d) {
            return Math.abs(((Twist2d) obj).dx - dx) < 1E-9
                && Math.abs(((Twist2d) obj).dy - dy) < 1E-9
                && Math.abs(((Twist2d) obj).dtheta - dtheta) < 1E-9;
        }
        return false;
    }

    @NonNull
    @Override
    public String toString() {
        return String.format(Locale.US,
                "Twist2d(dX: %.2f, dY: %.2f, dTheta: %.2f)", dx, dy, dtheta);
    }

    @Override
    public int hashCode() {
        return Objects.hash(dx, dy, dtheta);
    }
}
