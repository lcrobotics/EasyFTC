package com.lcrobotics.easyftclib.commandCenter.driveTrain.controller;

import com.lcrobotics.easyftclib.tools.MathUtils;

/**
 * PID controller for robots (https://en.wikipedia.org/wiki/PID_controller).
 * does all of the calculation but coefficients will vary from robot to robot.
 *
 * Equation:
 * u(t) = kP * e(t) + kI * int(0,t)[e(t')dt'] + kD * e'(t) + kF
 * where e(t) = r(t) - y(t), r(t) is the desired setpoint, and y(t) is the
 * measured value. If we consider e(t) to be the positional error, then
 * int(0,t)[e(t')dt'] is the total error and e'(t) is the velocity error.
 */
public class PIDFController {
    // coefficients of controller
    private double kP, kI, kD, kF;

    private double setPoint;
    private double measuredValue;
    private double minIntegral, maxIntegral;

    private double errorVal_p;
    private double errorVal_v;

    private double totalError;
    private double prevErrorVal;

    private double errorTolerance_p = 0.05;
    private double errorTolerance_v = Double.POSITIVE_INFINITY;

    private double lastTimeStamp;
    private double period;

    /**
     * Base constructor
     */
    public PIDFController(double kp, double ki, double kd, double kf) {
        this(kp, ki, kd, kf, 0, 0);
    }

    public PIDFController(double kp, double ki, double kd, double kf, double sp, double pv) {
        kP = kp;
        kI = ki;
        kD = kd;
        kF = kf;

        setPoint = sp;
        measuredValue = pv;

        minIntegral = -1;
        maxIntegral = 1;

        lastTimeStamp = 0;
        period = 0;

        errorVal_p = setPoint - measuredValue;
        reset();
    }

    public void reset() {
        totalError = 0;
        prevErrorVal = 0;
        lastTimeStamp = 0;
    }

    /**
     * Sets the error which is considered tolerable for use with {@link #atSetPoint()}.
     *
     * @param positionTolerance Position error which is tolerable.
     */
    public void setTolerance(double positionTolerance) {
        setTolerance(positionTolerance, Double.POSITIVE_INFINITY);
    }

    /**
     * Sets the error which is considered tolerable for use with {@link #atSetPoint()}.
     *
     * @param positionTolerance Position error which is tolerable.
     * @param velocityTolerance Velocity error which is tolerable.
     */
    public void setTolerance(double positionTolerance, double velocityTolerance) {
        errorTolerance_p = positionTolerance;
        errorTolerance_v = velocityTolerance;
    }

    /**
     * Returns the current setpoint of controller
     *
     * @return The current setpoint
     */
    public double getSetPoint() {
        return setPoint;
    }
    /**
     * Sets the setpoint for the PIDFController
     *
     * @param sp The desired setpoint.
     */
    public void setSetPoint(double sp) {
        setPoint = sp;
    }

    /**
     * Returns true if the error is within the percentage of the total input range,
     * determined by {@link #setTolerance}
     *
     * @return Whether the error is within the acceptable bounds
     */
    public boolean atSetPoint() {
        return Math.abs(errorVal_p) < errorTolerance_p
                && Math.abs(errorVal_v) < errorTolerance_v;
    }

    /**
     * @return The PIDF coefficients
     */
    public double[] getCoefficients() {
        return new double[]{kP, kI, kD, kF};
    }

    /**
     * @return The positional error e(t)
     */
    public double getPositionError() {
        return errorVal_p;
    }

    /**
     * @return The tolerances of the controller
     */
    public double[] getTolerance() {
        return new double[]{errorTolerance_p, errorTolerance_v};
    }

    /**
     * @return The velocity error e'(t)
     */
    public double getVelocityError() {
        return errorVal_v;
    }

    /**
     * Calculates the next output of the PIDF controller.
     *
     * @return the next output using the current measured value via
     * {@link #calculate(double)}.
     */
    public double calculate() {
        return calculate(measuredValue);
    }

    /**
     * Calculates the next output of the PIDF controller.
     *
     * @param pv The given measured value.
     * @param sp The given setpoint.
     * @return the next output using the given measurd value via
     * {@link #calculate(double)}.
     */
    public double calculate(double pv, double sp) {
        // set the setpoint to the provided value
        setSetPoint(sp);
        return calculate(pv);
    }

    /**
     * Calculates the control value, u(t).
     *
     * @param pv The current measurement of process variable.
     * @return The valuie produced by u(t).
     */
    public double calculate(double pv) {
        prevErrorVal = errorVal_p;

        double currentTimeStamp = (double) System.nanoTime() / 1E9;

        if (lastTimeStamp == 0) {
            lastTimeStamp = currentTimeStamp;
        }
        period = currentTimeStamp - lastTimeStamp;
        lastTimeStamp = currentTimeStamp;

        // calculate new positional error
        errorVal_p = setPoint - pv;
        measuredValue = pv;
        // only calculate velocity error if given enough of a period
        if (Math.abs(period) > 1E-6) {
            errorVal_v = (errorVal_p - prevErrorVal) / period;
        } else {
            errorVal_v = 0;
        }

        totalError = period * (setPoint - measuredValue);
        // clamp between min and max
        totalError = MathUtils.clamp(totalError, minIntegral, maxIntegral);

        // calculate u(t)
        return kP * errorVal_p + kI * totalError + kD * errorVal_v + kF * setPoint;
    }

    public void setPIDF(double kp, double ki, double kd, double kf) {
        kP = kp;
        kI = ki;
        kD = kd;
        kF = kf;
    }

    public void setIntegrationBounds(double integralMin, double integralMax) {
        minIntegral = integralMin;
        maxIntegral = integralMax;
    }

    // used to clear kI gains
    public void clearTotalError() {
        totalError = 0;
    }

    public void setP(double kp) {
        kP = kp;
    }

    public void setI(double ki) {
        kI = ki;
    }

    public void setD(double kd) {
        kD = kd;
    }

    public void setF(double kf) {
        kF = kf;
    }

    public double getP() {
        return kP;
    }

    public double getI() {
        return kI;
    }

    public double getD() {
        return kD;
    }

    public double getF() {
        return kF;
    }

    public double getPeriod() {
        return period;
    }
}
