package com.lcrobotics.easyftclib.CommandCenter.driveTrain;

import com.qualcomm.robotcore.hardware.DcMotor;

public class SmartMotor extends DriveMotor {
    private final int COUNTSPERREV = 28;
    double wheelRadius;
    int reduction;
    double cmpercount;

    public SmartMotor(DcMotor motor, WheelPosition motorPosition, int reduction, int wheelRadius) {
        super(motor, motorPosition);
        this.reduction = reduction;
        this.wheelRadius = wheelRadius;
        this.cmpercount = (2 * wheelRadius * Math.PI) / (COUNTSPERREV * reduction);
    }

    public SmartMotor(DcMotor motor, WheelPosition motorPosition, int reduction) {
        this(motor, motorPosition, reduction, 5);
    }

    public void drive(double distance, double power) {
        double counts = distance / cmpercount;
        this.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.motor.setTargetPosition((int)Math.round(counts) + this.motor.getCurrentPosition());
        this.motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.motor.setPower(power);
    }

    public int getReduction() {
        return reduction;
    }

    public void setReduction(int reduction) {
        this.reduction = reduction;
    }

    public int getCount() {
        return this.motor.getCurrentPosition();
    }

    public int getTarget(double distance) {
        return (int)Math.round(distance / cmpercount);
    }
}
