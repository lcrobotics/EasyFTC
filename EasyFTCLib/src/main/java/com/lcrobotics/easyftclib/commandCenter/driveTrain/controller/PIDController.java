package com.lcrobotics.easyftclib.commandCenter.driveTrain.controller;

public class PIDController extends PIDFController {
    /**
     * Base constructor
     */
    public PIDController(double kp, double ki, double kd) {
        super(kp, ki, kd, 0);
    }

    /**
     * Extended constructor
     */
    public PIDController(double kp, double ki, double kd, double sp, double pv) {
        super(kp, ki, kd, 0, sp, pv);
    }

    public void setPID(double kp, double ki, double kd) {
        setPIDF(kp, ki, kd, 0);
    }
}
