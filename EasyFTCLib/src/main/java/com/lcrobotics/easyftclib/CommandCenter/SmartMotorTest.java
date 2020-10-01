package com.lcrobotics.easyftclib.CommandCenter;

import com.lcrobotics.easyftclib.AdvancedOpMode;
import com.lcrobotics.easyftclib.CommandCenter.driveTrain.SmartMotor;
import com.lcrobotics.easyftclib.CommandCenter.driveTrain.WheelPosition;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous
public class SmartMotorTest extends AdvancedOpMode {
    SmartMotor frontRightDrive;
    SmartMotor frontLeftDrive;
    SmartMotor backLeftDrive;
    SmartMotor backRightDrive;
    boolean once = true;
    int targetVal;

    @Override
    public void init() {
        frontLeftDrive = new SmartMotor(hardwareMap.get(DcMotor.class, "FrontLeftDrive"), WheelPosition.FRONT_LEFT, 16);
        frontRightDrive = new SmartMotor(hardwareMap.get(DcMotor.class, "FrontRightDrive"), WheelPosition.FRONT_RIGHT, 16);
        backLeftDrive = new SmartMotor(hardwareMap.get(DcMotor.class, "BackLeftDrive"), WheelPosition.BACK_LEFT, 16);
        backRightDrive = new SmartMotor(hardwareMap.get(DcMotor.class, "BackRightDrive"), WheelPosition.BACK_RIGHT, 16);
        targetVal = backLeftDrive.getTarget(20);
    }

    @Override
    public void loop() {
        if (once) {
            frontLeftDrive.drive(20, 0.5);
            frontRightDrive.drive(-20, 0.5);
            backLeftDrive.drive(20, 0.5);
            backRightDrive.drive(-20, 0.5);
            once = false;
        }
        double encoderVal = backLeftDrive.getCount();
        telemetry.addData("backLeftDrive Encoder Value:", encoderVal);
        telemetry.addData("target: ", targetVal);
        telemetry.update();
    }
}
