package com.lcrobotics.easyftclib.tools.geometry;

import android.support.annotation.NonNull;
import android.support.annotation.Nullable;

import java.util.Locale;
import java.util.Objects;

/** Represents a 2d pose containing translational and rotational elements. */
public class Pose2d {
    private final Translation2d translation;
    private final Rotation2d rotation;

    public Pose2d() {
        translation = new Translation2d();
        rotation = new Rotation2d();
    }

    /**
     * Constructs a pose with the given translation and rotation.
     *
     * @param translation The translational component of the pose.
     * @param rotation The rotational component of the pose.
     */
    public Pose2d(Translation2d translation, Rotation2d rotation) {
        this.translation = translation;
        this.rotation = rotation;
    }

    /**
     * Convenience constructor that takes in x and y values directly instead of having
     * to construct a Translation2d
     * @param x The x component of the translation.
     * @param y The y component of the translation.
     * @param rotation The rotational component of the pose.
     */
    public Pose2d(double x, double y, Rotation2d rotation) {
        translation = new Translation2d(x, y);
        this.rotation = rotation;
    }
    /**
     * Transforms the pose by the given transformation and returns the new
     * transformed pose.
     *
     * <p>The matrix multiplication is as follows
     * [x_new]    [cos, -sin, 0][transform.x]
     * [y_new] += [sin,  cos, 0][transform.y]
     * [t_new]    [0,    0,   1][transform.t]
     *
     * @param other The transform to transform the pose by.
     * @return The transformed pose.
     */
    public Pose2d plus(Transform2d other) {
        return transformBy(other);
    }
    /**
     * Returns the Transform2d that maps the one pose to another.
     *
     * @param other The initial pose of the transformation.
     * @return The transform that maps the other pose to the current pose.
     */
    public Transform2d minus(Pose2d other) {
        Pose2d temp = relativeTo(other);
        return new Transform2d(temp.getTranslation(), temp.getRotation());
    }
    /**
     * @return The translational component of the pose.
     */
    public Translation2d getTranslation() {
        return translation;
    }

    /**
     * @return The x component of the pose's translation
     */
    public double getX() {
        return translation.getX();
    }

    /**
     * @return The y component of the pose's translation
     */
    public double getY() {
        return translation.getY();
    }

    /**
     * @return The rotational component of the pose.
     */
    public Rotation2d getRotation() {
        return rotation;
    }

    /**
     * Transforms the pose by the given transformation and returns the new pose. See + operator for
     * the matrix multiplication performed.
     *
     * @param other The transform to transform the pose by.
     * @return The transformed pose.
     */
    public Pose2d transformBy(Transform2d other) {
        return new Pose2d(
                translation.plus(other.getTranslation().rotateBy(rotation)),
                rotation.plus(other.getRotation())
        );
    }

    /**
     * Returns the other pose relative to the current pose.
     *
     * @param other The pose that is the origin of the new coordinate frame that the current pose
     *              will be converted into.
     * @return The current pose relative to the new origin pose.
     */
    public Pose2d relativeTo(Pose2d other) {
        Transform2d temp = new Transform2d(other, this);
        return new Pose2d(temp.getTranslation(), temp.getRotation());
    }
    /**
     * Obtain a new Pose2d from a (constant curvature) velocity.
     *
     * <p>See <a href="https://file.tavsys.net/control/controls-engineering-in-frc.pdf">Controls
     * Engineering in the FIRST Robotics Competition</a> section 10.2 "Pose exponential" for a
     * derivation.
     *
     * <p>The twist is a change in pose in the robot's coordinate frame since the previous pose
     * update. When the user runs exp() on the previous known field-relative pose with the argument
     * being the twist, the user will receive the new field-relative pose.
     *
     * <p>"Exp" represents the pose exponential, which is solving a differential equation moving the
     * pose forward in time.
     *
     * @param twist The change in pose in the robot's coordinate frame since the previous pose update.
     *     For example, if a non-holonomic robot moves forward 0.01 meters and changes angle by 0.5
     *     degrees since the previous pose update, the twist would be Twist2d{0.01, 0.0,
     *     toRadians(0.5)}
     * @return The new pose of the robot.
     */
    public Pose2d exp(Twist2d twist) {
        double dx = twist.dx;
        double dy = twist.dy;
        double dtheta = twist.dtheta;

        double sinTheta = Math.sin(dtheta);
        double cosTheta = Math.cos(dtheta);

        double s;
        double c;
        // if change in angle is negligible
        if (Math.abs(dtheta) < 1E-9) {
            s = 1.0 - (dtheta * dtheta) / 6.0;
            c = 0.5 * dtheta;
        } else {
            s = sinTheta / dtheta;
            c = (1.0 - cosTheta) / dtheta;
        }

        Transform2d transform =
                new Transform2d(
                        new Translation2d(dx * s - dy * c, dx * c + dy * s),
                        new Rotation2d(cosTheta, sinTheta)
                );

        return this.plus(transform);
    }

    /**
     * Returns a Twist2d that maps this pose to the end pose. If c is the output of a.Log(b), then
     * a.Exp(c) would yield b.
     *
     * @param end The end pose for the transformation.
     * @return The twist that maps this to to end.
     */
    public Twist2d log(Pose2d end) {
        Pose2d transform = end.relativeTo(this);
        // convenience variables for shorthand
        double dtheta = transform.getRotation().getRadians();
        double cosMinusOne = transform.getRotation().getCos() - 1;
        double halfDtheta = dtheta / 2.0;

        double halfThetaByTanOfHalfDtheta;
        if (Math.abs(cosMinusOne) < 1E-9) {
            halfThetaByTanOfHalfDtheta = 1.0 - (dtheta * dtheta) / 12.0;
        } else {
            halfThetaByTanOfHalfDtheta = -(halfDtheta * transform.getRotation().getSin()) / cosMinusOne;
        }

        Translation2d translationPart =
                transform.getTranslation()
                         .rotateBy(new Rotation2d(halfThetaByTanOfHalfDtheta, -halfDtheta))
                         .times(Math.hypot(halfThetaByTanOfHalfDtheta, halfDtheta));

        return new Twist2d(translationPart.getX(), translationPart.getY(), dtheta);
    }
    @NonNull
    @Override
    public String toString() {
        return String.format(Locale.US, "Pose2d(%s, %s)", translation, rotation);
    }

    /**
     * Checks equality between this Pose2d and another object.
     *
     * @param obj The other object.
     * @return Whether the two objects are equal or not.
     */
    @Override
    public boolean equals(@Nullable Object obj) {
        if (obj instanceof Pose2d) {
            return ((Pose2d) obj).translation.equals(translation)
                && ((Pose2d) obj).rotation.equals(rotation);
        }
        return false;
    }



    @Override
    public int hashCode() {
        return Objects.hash(translation, rotation);
    }
}
