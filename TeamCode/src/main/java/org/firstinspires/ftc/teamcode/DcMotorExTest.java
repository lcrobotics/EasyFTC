package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

@Autonomous
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

        FrontLeftDrive.setPositionPIDFCoefficients(5);
        FrontRightDrive.setPositionPIDFCoefficients(5);
        BackLeftDrive.setPositionPIDFCoefficients(5);
        BackRightDrive.setPositionPIDFCoefficients(5);

        FrontLeftDrive.setVelocityPIDFCoefficients(2, 0.5, 11, 0);
        FrontRightDrive.setVelocityPIDFCoefficients(2, 0.5, 11, 0);
        BackLeftDrive.setVelocityPIDFCoefficients(2, 0.5, 11, 0);
        BackRightDrive.setVelocityPIDFCoefficients(2, 0.5, 11, 0);
    }

    @Override
    public void loop() {
        FrontLeftDrive.setVelocity(14.26);
        FrontRightDrive.setVelocity(14.26);
        BackLeftDrive.setVelocity(14.26);
        BackRightDrive.setVelocity(14.26);
    }
}
