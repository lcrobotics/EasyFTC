package com.lcrobotics.easyftclib.tools.geometry;

import android.support.annotation.NonNull;

import java.util.Objects;
/** Represents a transformation for a Pose2d. */
public class Transform2d {
    private final Translation2d translation;
    private final Rotation2d rotation;

    /**
     * Constructs the transform that maps the initial pose to the final pose.
     *
     * @param initial The initial pose.
     * @param last The final pose.
     */
    public Transform2d(Pose2d initial, Pose2d last) {
        // We are rotating the difference between the translations
        // using a clockwise rotation matrix. This transforms the global
        // delta into a local delta (relative to the initial pose).
        translation = last.getTranslation()
                          .minus(initial.getTranslation())
                          .rotateBy(initial.getRotation().unaryMinus());

        rotation = last.getRotation().minus(initial.getRotation());
    }
    /**
     * Constructs a transform with the given translation and rotation components.
     *
     * @param translation Translational component.
     * @param rotation Rotational component.
     */
    public Transform2d(Translation2d translation, Rotation2d rotation) {
        this.translation = translation;
        this.rotation = rotation;
    }

    /**
     * Constructs base transform
     */
    public Transform2d() {
        translation = new Translation2d();
        rotation = new Rotation2d();
    }

    public Transform2d times(double scalar) {
        return new Transform2d(translation.times(scalar), rotation.times(scalar));
    }

    /**
     * @return The translational component of the transform.
     */
    public Translation2d getTranslation() {
        return translation;
    }
    /**
     * @return The x component of the transformation's translation.
     */
    public double getX() {
        return translation.getX();
    }

    /**
     * @return The y component of the transformation's translation.
     */
    public double getY() {
        return translation.getY();
    }
    /**
     * @return The rotational component of the transform.
     */
    public Rotation2d getRotation() {
        return rotation;
    }

    /**
     * Invert the transformation. This is useful for undoing a transformation.
     *
     * @return The inverted transformation
     */
    public Transform2d inverse() {
        return new Transform2d(
                translation.unaryMinus().rotateBy(getRotation().unaryMinus()),
                getRotation().unaryMinus()
        );
    }
    @NonNull
    @Override
    public String toString() {
        return String.format("Transform2d(%s, %s)", translation, rotation);
    }

    /**
     * Checks equality between this Transform2d and another object.
     *
     * @param obj The other object.
     * @return Whether the two objects are equal.
     */
    @Override
    public boolean equals(Object obj) {
        if (obj instanceof Transform2d) {
            return ((Transform2d) obj).translation.equals(translation)
                    && ((Transform2d) obj).rotation.equals(rotation);
        }
        return false;
    }

    @Override
    public int hashCode() {
        return Objects.hash(translation, rotation);
    }
}
