package com.lcrobotics.easyftclib.commandCenter.driveTrain.controller;

public class PDController extends PIDController {

    /**
     * Base constructor
     */
    public PDController(double kp, double kd){
        super(kp, 0, kd);
    }

    /**
     * Extended constructor.
     */
    public PDController(double kp, double kd, double sp, double pv) {
        super(kp, 0, kd, sp, pv);
    }
}
