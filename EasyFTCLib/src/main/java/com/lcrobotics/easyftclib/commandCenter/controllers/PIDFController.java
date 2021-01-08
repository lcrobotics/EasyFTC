package com.lcrobotics.easyftclib.commandCenter.controllers;

public class PIDFController {

    private double kP, kI, kD, kF;
    private double setPoint;
    private double measuredValue;
    private double minIntegral, maxIntegral;

    private double errorPos;
    private double errorVel;

    private double totalError;
    private double prevError;

    private double errorPosTolerance = 0.05;
    private double errorVelTolerance = Double.POSITIVE_INFINITY;

    private double lastTimeStamp;
    private double period;

    /**
     * base constructor
     */
    public PIDFController(double kp, double ki, double kd, double kf) {

    }
    /**
     * full constructor
     *
     * @param sv setpoint of PIDF control loop.
     * @param mv measured value of control loop.
     */
    public PIDFController(double kp, double ki, double kd, double kf, double sv, double mv) {
        kP = kp;
        kI = ki;
        kD = kd;
        kF = kf;

        setPoint = sv;
        measuredValue = mv;

        minIntegral = -1.0;
        maxIntegral = 1.0;

        lastTimeStamp = 0;
        period = 0;

        errorPos = setPoint - measuredValue;
        reset();
    }

    public void reset() {
        totalError = 0;
        prevError = 0;
        lastTimeStamp = 0;
    }

    /**
     * Sets bound for which error is acceptable when using {@link #atSetPoint()}
     *
     * @param positionTolerance Position tolerance
     */
    public void setTolerance(double positionTolerance) {
        setTolerance(positionTolerance, Double.POSITIVE_INFINITY);
    }
    /**
     * Sets bound for which error is acceptable when using {@link #atSetPoint()}.
     *
     * @param positionTolerance Position tolerance
     * @param velocityTolerance Velocity tolerance
     */
    public void setTolerance(double positionTolerance, double velocityTolerance) {
        errorPosTolerance = positionTolerance;
        errorVelTolerance = velocityTolerance;
    }

    /**
     * Gets set point of PIDF controller.
     * @return set point of controller
     */
    public double getSetPoint() {
        return setPoint;
    }

    /**
     * Sets set point of PIDF controller.
     * @param setPoint set point of controller
     */
    public void setSetPoint(double setPoint) {
        this.setPoint = setPoint;
    }
    /**
     * Returns true if the error is within acceptable bounds, determined by
     * {@link #setTolerance}.
     *
     * @return Whether the error is within the acceptable bounds.
     */
    public boolean atSetPoint() {
        return Math.abs(errorPos) < errorPosTolerance && Math.abs(errorVel) < errorVelTolerance;
    }
    /**
     * @return PIDF coefficients
     */
    public double[] getCoefficients() {
        return new double[]{kP, kI, kD, kF};
    }
    /**
     * @return the position error
     */
    public double getPositionError() {
        return errorPos;
    }
    /**
     * @return the velocity error
     */
    public double getVelocityError() {
        return errorVel;
    }
    /**
     * @return the tolerances of the controller
     */
    public double[] getTolerances() {
        return new double[]{errorPosTolerance, errorVelTolerance};
    }

    /**
     * Calculates the next output of the PIDF controller.
     *
     * @return the next output using the current measured value via {@link #calculate(double)}.
     */
    public double calculate() {
        return calculate(measuredValue);
    }

    /**
     * Calculates the next output of the PIDF controller.
     *
     * @param mv The given measured value
     * @param sp The given set point
     * @return the next output using the given measured value via {@link #calculate(double)}.
     */
    public double calculate(double mv, double sp) {
        setSetPoint(sp);
        return calculate(mv);
    }

    /**
     * Calculates the next output of the PIDF controller, u(t).
     *
     * @param mv The given measured value.
     * @return the value produced by u(t)
     */
    public double calculate(double mv) {
        prevError = errorPos;
        // calculate time between calls
        double currentTimeStamp = System.nanoTime() / 1E9;

        if (lastTimeStamp == 0) {
            period = 0;
        } else {
            period = currentTimeStamp - lastTimeStamp;
        }
        lastTimeStamp = currentTimeStamp;
        // calculate error
        errorPos = setPoint - mv;

        measuredValue = mv;

        if (Math.abs(period) > 1E-6) {
            errorVel = (errorPos - prevError) / period;
        } else {
            errorVel = 0;
        }

        totalError = period * (setPoint - measuredValue);
        // clamp between minIntegral and maxIntegral
        totalError = Math.max(minIntegral, Math.min(totalError, maxIntegral));

        return kP * errorPos + kI * totalError + kD * errorVel + kF * setPoint;
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

    public void clearTotalError() {
        totalError = 0;
    }

    public void setP(double kP) {
        this.kP = kP;
    }

    public void setI(double kI) {
        this.kI = kI;
    }

    public void setD(double kD) {
        this.kD = kD;
    }

    public void setF(double kF) {
        this.kF = kF;
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
