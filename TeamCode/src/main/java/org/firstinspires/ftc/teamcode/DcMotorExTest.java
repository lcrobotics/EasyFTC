package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

public class DcMotorExTest extends OpMode {

    public DcMotorEx FrontLeftDrive = null;
    public DcMotorEx FrontRightDrive = null;
    public DcMotorEx BackLeftDrive = null;
    public DcMotorEx BackRightDrive = null;


    @Override
    public void init() {
        FrontLeftDrive = (DcMotorEx)hardwareMap.get(DcMotor.class, "FrontLeftDrive");
        FrontRightDrive = (DcMotorEx)hardwareMap.get(DcMotor.class, "FrontRightDrive");
        BackLeftDrive = (DcMotorEx)hardwareMap.get(DcMotor.class, "BackLeftDrive");
        BackRightDrive = (DcMotorEx)hardwareMap.get(DcMotor.class, "BackRightDrive");

        FrontLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        FrontRightDrive.setDirection(DcMotor.Direction.REVERSE);
        BackLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        BackRightDrive.setDirection(DcMotor.Direction.REVERSE);
    }

    @Override
    public void loop() {
    }
}
