package com.lcrobotics.easyftclib.tools.geometry;

import android.support.annotation.NonNull;
import android.support.annotation.Nullable;

import java.util.Locale;
import java.util.Objects;

/** A rotation in a 2d coordinate frame represented as
 *  a point on the unit circle (cosine and sine).
 */
public class Rotation2d {
    private final double value;
    private final double cos;
    private final double sin;

    /**
     * Constructs a Rotation2d with a default angle of 0 degrees.
     */
    public Rotation2d() {
        value = 0;
        cos = 1;
        sin = 0;
    }

    /**
     * Constructs a Rotation2d with the given angle value.
     *
     * @param value The angle in radians.
     */
    public Rotation2d(double value) {
        this.value = value;
        cos = Math.cos(value);
        sin = Math.sin(value);
    }

    /**
     * Constructs a Rotation2d with the given x and y components.
     *
     * @param x The x component or cosine of the rotation.
     * @param y The y component or sin of the rotation.
     */
    public Rotation2d(double x, double y) {
        double magnitude = Math.hypot(x, y);
        // very small magnitudes are treated as 0
        if (magnitude > 1E-6) {
            sin = y / magnitude;
            cos = x / magnitude;
        } else {
            sin = 0.0;
            cos = 1.0;
        }
        value = Math.atan2(sin, cos);
    }

    /**
     * Constructs a Rotation2d with the given angle in degrees.
     *
     * @param degrees The angle in degrees.
     * @return A Rotation2d object with the given angle.
     */
    public static Rotation2d fromDegrees(double degrees) {
        return new Rotation2d(Math.toRadians(degrees));
    }

    /**
     * Adds two rotations together.
     *
     * @param other The rotation to add.
     * @return The sum of the two rotations.
     */
    public Rotation2d plus(Rotation2d other) {
        return rotateBy(other);
    }

    /**
     * Subtracts the new rotation from the current rotation and returns the result.
     *
     * @param other The rotation to subtract.
     * @return The difference between the two rotations.
     */
    public Rotation2d minus(Rotation2d other) {
        return rotateBy(other.unaryMinus());
    }

    /**
     * Takes the inverse of the current rotation by negating the angle.
     *
     * @return The inverse of the current rotation.
     */
    public Rotation2d unaryMinus() {
        return new Rotation2d(-value);
    }

    /**
     * Multiplies the current rotation by a scalar.
     *
     * @param scalar The scalar.
     * @return The scaled rotation.
     */
    public Rotation2d times(double scalar) {
        return new Rotation2d(value * scalar);
    }
    /**
     * Combines the two rotations using rotation matrices and matrix multiplication
     *
     * @param other The rotation to rotate by.
     * @return The new rotated Rotation2d.
     */
    public Rotation2d rotateBy(Rotation2d other) {
        return new Rotation2d(
                cos * other.cos - sin * other.sin, cos * other.sin + sin * other.cos
        );
    }

    /**
     * @return The radian value of the rotation
     */
    public double getRadians() {
        return value;
    }

    /**
     * @return The degree value of the rotation.
     */
    public double getDegrees() {
        return Math.toDegrees(value);
    }

    /**
     * @return The cosine of the rotation
     */
    public double getCos() {
        return cos;
    }
    /**
     * @return The sine of the rotation
     */
    public double getSin() {
        return sin;
    }

    /**
     * @return The tangent of the rotation.
     */
    public double getTan() {
        return sin / cos;
    }

    @NonNull
    @Override
    public String toString() {
        return String.format(Locale.US, "Rotation2d(Rads: %.2f, Deg: %.2f)", value, Math.toDegrees(value));
    }

    /**
     * Checks equality between this rotation and another object.
     *
     * @param obj The other object.
     * @return Whether the two objects are equal.
     */
    @Override
    public boolean equals(@Nullable Object obj) {
        if (obj instanceof Rotation2d) {
            return Math.abs(((Rotation2d) obj).value - value) < 1E-9;
        }
        return false;
    }

    @Override
    public int hashCode() {
        return Objects.hash(value);
    }
}
