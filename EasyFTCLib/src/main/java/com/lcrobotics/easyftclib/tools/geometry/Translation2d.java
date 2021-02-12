package com.lcrobotics.easyftclib.tools.geometry;

import android.support.annotation.NonNull;

import java.util.Locale;
import java.util.Objects;
/**
 * Represents a translation in 2d space. This object can be used to represent a point or a vector.
 *
 * <p>This assumes that you are using conventional mathematical axes. When the robot is placed on
 * the origin, facing toward the X direction, moving forward increases the X, whereas moving to the
 * left increases the Y.
 */
public class Translation2d {
    private final double x;
    private final double y;

    /**
     * Constructs translation with x and y components equal to 0
     */
    public Translation2d() {
        this(0.0, 0.0);
    }

    /**
     * Constructs translation with x and y components equal to provided values
     *
     * @param x The x component of the translation.
     * @param y The y component of the translation.
     */
    public Translation2d(double x, double y) {
        this.x = x;
        this.y = y;
    }

    /**
     * Calculates distance between two translations in 2d space.
     *
     * @param other The other translation.
     * @return The distance between the two translations.
     */
    public double getDistance(Translation2d other) {
        return Math.hypot(other.x - x, other.y - y);
    }

    /**
     * @return The x component of the translation
     */
    public double getX() {
        return x;
    }

    /**
     * @return The y component of the translation
     */
    public double getY() {
        return y;
    }

    /**
     * Calculates the norm (distance from the origin) of the translation.
     *
     * @return The norm of the translation.
     */
    public double getNorm() {
        return Math.hypot(x, y);
    }
    /**
     * Applies a rotation to the translation.
     *
     * @param other The rotation to rotate the translation by.
     * @return The new rotated translation.
     */
    public Translation2d rotateBy(Rotation2d other) {
        return new Translation2d(
                x * other.getCos() - y * other.getSin(), x * other.getSin() + y * other.getCos()
        );
    }
    /**
     * Adds two translations in 2d space.
     *
     * @param other The other translation.
     * @return The resulting translation.
     */
    public Translation2d plus(Translation2d other) {
        return new Translation2d(x + other.x, y + other.y);
    }

    /**
     * Subtracts the other translation from this translation in 2d space.
     *
     * @param other The other translation.
     * @return The resulting translation.
     */
    public Translation2d minus(Translation2d other) {
        return new Translation2d(x - other.x, y - other.y);
    }

    /**
     * Returns the inverse of the current translation.
     *
     * @return The inverse.
     */
    public Translation2d unaryMinus() {
        return new Translation2d(-x, -y);
    }

    /**
     * Multiplies the translation by a scalar
     *
     * @param scalar The scalar to multiply by.
     * @return The scaled translation.
     */
    public Translation2d times(double scalar) {
        return new Translation2d(x * scalar, y * scalar);
    }

    /**
     * Divides the translation by a scalar.
     *
     * @param scalar The scalar to divide by.
     * @return The scaled translation.
     */
    public Translation2d div(double scalar) {
        return new Translation2d(x / scalar, y / scalar);
    }

    @NonNull
    @Override
    public String toString() {
        return String.format(Locale.US, "Translation2d(X: %.2f, Y: %.2f)", x, y);
    }

    /**
     * Checks equality between this translation and another object.
     *
     * @param obj The other object.
     * @return Whether the two objects are equal.
     */
    @Override
    public boolean equals(Object obj) {
        if (obj instanceof Translation2d) {
            return Math.abs(((Translation2d) obj).x - x) < 1E-9
                && Math.abs(((Translation2d) obj).y - y) < 1E-9;
        }
        return false;
    }

    @Override
    public int hashCode() {
        return Objects.hash(x, y);
    }
}
