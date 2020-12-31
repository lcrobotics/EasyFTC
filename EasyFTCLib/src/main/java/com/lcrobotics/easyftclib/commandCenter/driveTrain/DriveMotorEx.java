package com.lcrobotics.easyftclib.commandCenter.driveTrain;

import com.qualcomm.robotcore.hardware.DcMotorEx;

public class DriveMotorEx {
    // FTC motor class
    DcMotorEx motor;
    // Stores whether the motor is left or right (and maybe front or back)
    WheelPosition position;
    String motorName;
    // Stores whether the motor power has changed and needs to be set again by the OpMode
    boolean needsUpdate;

    /**
     * Constructor that includes a motor and a position
     * @param motor
     * @param motorPosition
     */
    public DriveMotorEx(DcMotorEx motor, WheelPosition motorPosition){
        this.motor = motor;
        this.position = motorPosition;
        this.motorName = motor.getDeviceName();
        this.needsUpdate = false;
    }

    /**
     * Constructor that doesn't require a particular motor
     * Object serves as a placeholder until a motor can be connected with setMotor()
     * @param motorName
     * @param motorPosition
     */
    public DriveMotorEx(String motorName, WheelPosition motorPosition) {
        this.motorName = motorName;
        this.needsUpdate = true;
        this.position = motorPosition;
    }

    // Getters and setters

    public DcMotorEx getMotor() {
        return motor;
    }

    public void setMotor(DcMotorEx motor) {
        this.motor = motor;
    }

    public void setPosition(WheelPosition position) {
        this.position = position;
    }

    public WheelPosition getPosition() {
        return position;
    }
}
