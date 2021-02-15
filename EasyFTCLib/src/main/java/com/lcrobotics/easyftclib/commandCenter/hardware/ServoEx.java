package com.lcrobotics.easyftclib.CommandCenter.hardware;

public interface ServoEx extends HardwareDevice {
    /**
     * Rotates the servo a certain amount of degrees
     *
     * @param degrees The amount of degrees to rotate
     */
    void rotateDegrees(double degrees);

    /**
     * Rotates the servo to a specific angle
     *
     * @param angle The desired angle to rotate to
     */
    void turnToAngle(double angle);

    /**
     * Rotates the servo by given positional factor
     *
     * @param position The positional factor to rotate by
     */
    void rotate(double position);

    /**
     * Sets range of servo
     *
     * @param min minimum value. Setting the position to 0 will make it go to this value.
     * @param max maximum value. Setting the position to 1 will make it go to this value.
     */
    void setRange(double min, double max);

    /**
     * @return true if servo is inverted, false otherwise.
     */
    boolean getInverted();

    /**
     * Sets inversion of servo. By default, this is false.
     *
     * @param inverted The desired inversion.
     */
    void setInverted(boolean inverted);

    /**
     * @return current position of servo from 0 to 1.
     */
    double getPosition();

    /**
     * Sets target position of servo to given position
     *
     * @param position The target position, ranging from 0 to 1
     */
    void setPosition(double position);

    /**
     * @return angle of servo relative ot the 0 position.
     */
    double getAngle();
}
