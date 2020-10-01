package com.lcrobotics.easyftclib.CommandCenter.driveTrain;

import com.qualcomm.robotcore.hardware.DcMotor;

public class SmartMotor extends DriveMotor {
    int reduction;
    double cmpercount;

    public SmartMotor(DcMotor motor, WheelPosition motorPosition, int reduction) {
        super(motor, motorPosition);
        this.reduction = reduction;
        this.cmpercount = (5 * 3.1415) / (14 * reduction);
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
