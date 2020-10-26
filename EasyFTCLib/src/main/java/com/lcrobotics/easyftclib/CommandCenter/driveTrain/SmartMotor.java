package com.lcrobotics.easyftclib.CommandCenter.driveTrain;

import com.lcrobotics.easyftclib.AdvancedOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

// SmartMotors use encoders to rotate a certain distance
public class SmartMotor extends DriveMotor {
    // encoder counts per revolution
    private final int COUNTSPERREV = 28;
    // radius of the wheel
    double wheelRadius;
    // reduction ratio of the motor
    int reduction;
    // encoder counts per cm
    double countPerCm;

    // constructor that asks for wheelRadius
    public SmartMotor(DcMotor motor, WheelPosition motorPosition, int reduction, int wheelRadius) {
        // initialize DriveMotor
        super(motor, motorPosition);
        // update reduction and wheelRadius
        this.reduction = reduction;
        this.wheelRadius = wheelRadius;
        // calculate encoder counts per cm
        this.countPerCm = (COUNTSPERREV * reduction) / (2 * wheelRadius * Math.PI);
    }

    // constructor that assumes wheelRadius = 5
    public SmartMotor(DcMotor motor, WheelPosition motorPosition, int reduction) {
        this(motor, motorPosition, reduction, 5);
    }
    // rotates the motor the given distance with the given power
    public void drive(double distance, double power) {

        // calculate number of encoder counts required to rotate given distance
        double counts = distance * countPerCm;
        // reset motor's encoder
        this.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // set target position for motor's encoder
        this.motor.setTargetPosition((int)Math.round(counts) + this.motor.getCurrentPosition());
        this.motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // run motor
        this.motor.setPower(power);
    }
    public boolean isBusy() {
        return Math.abs(this.motor.getTargetPosition() - this.motor.getCurrentPosition()) > 13;
    }
    public int getReduction() {
        return reduction;
    }

    public void setReduction(int reduction) {
        this.reduction = reduction;
    }

    public int getCurrentCount() {
        return this.motor.getCurrentPosition();
    }

    public int getTarget(double distance) {
        return (int)Math.round(distance * countPerCm);
    }
}
