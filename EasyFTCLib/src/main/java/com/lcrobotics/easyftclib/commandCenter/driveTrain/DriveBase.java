
package com.lcrobotics.easyftclib.commandCenter.driveTrain;

import com.lcrobotics.easyftclib.tools.MathUtils;

/**
 * Base class for all drivebases, users should only use this when defining their own drivebases.
 */
public abstract class DriveBase {

    public static final double defaultRangeMin = -1.0;
    public static final double defaultRangeMax = 1.0;
    public static final double defaultMaxSpeed = 1.0;

    protected double rangeMin = defaultRangeMin;
    protected double rangeMax = defaultRangeMax;
    protected double maxSpeed = defaultMaxSpeed;

    public DriveBase() {

    }

    /**
     * Sets max output allowed for this drive train.
     * Outputs will be scaled to this value.
     *
     * <p>Defaults to {@value #defaultMaxSpeed}</p>
     *
     * @param max The max speed for drive functions.
     */
    public void setMaxSpeed(double max) {
        maxSpeed = max;
    }

    /**
     * Sets range for inputs to drive functions.
     *
     * <p>Defaults to the range [{@value #defaultRangeMin}, {@value #defaultRangeMax}].
     * Inputs are clipped to this range, see {@link #clipRange}</p>
     *
     * @param min The minimum of the range.
     * @param max The maximum of the range.
     */
    public void setRange(double min, double max) {
        rangeMin = min;
        rangeMax = max;
    }

    /**
     * Clamps/clips value to within the range [{@link #rangeMin}, {@link #rangeMax}].
     *
     * @param value Value to be clipped
     * @return The clipped value
     */
    public double clipRange(double value) {
        return MathUtils.clamp(value, rangeMin, rangeMax);
    }

    public abstract void stop();
    /**
     * Normalize wheel speeds, scaling the max speed down to the provided magnitude
     * while keeping the ratios in between the speeds the same
     *
     * @param wheelSpeeds wheel speeds to normalize
     * @param magnitude max speed allowed
     */
    protected void normalize(double[] wheelSpeeds, double magnitude) {
        double max = Math.abs(wheelSpeeds[0]);
        // find max wheel speed
        for (double x : wheelSpeeds) {
            double temp = Math.abs(x);
            if (temp > max) {
                max = temp;
            }
        }
        // normalize speeds
        for (int i = 0; i < wheelSpeeds.length; i++) {
            wheelSpeeds[i] = (wheelSpeeds[i] / max) * magnitude;
        }
    }

    /**
     * Normalize wheel speeds, scaling the max speed down to 1 while keeping the ratios
     * in between the speeds the same
     *
     * @param wheelSpeeds wheel speeds to normalize
     */
    protected void normalize(double[] wheelSpeeds) {
        double max = Math.abs(wheelSpeeds[0]);
        // find max wheel speed
        for (double x : wheelSpeeds) {
            double temp = Math.abs(x);
            if (temp > max) {
                max = temp;
            }
        }
        if (max > 1) {
            // normalize speeds
            for (int i = 0; i < wheelSpeeds.length; i++) {
                wheelSpeeds[i] = (wheelSpeeds[i] / max);
            }
        }
    }

    /**
     * Square input while keeping the sign
     */
    protected double square(double input) {
        return (input * input) * Math.signum(input);
    }
}