package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Autonomous
public class DcMotorExTest extends OpMode {
    // declare drive motors
    public DcMotorEx FrontLeftDrive = null;
    public DcMotorEx FrontRightDrive = null;
    public DcMotorEx BackLeftDrive = null;
    public DcMotorEx BackRightDrive = null;


    @Override
    public void init() {
        // initialize drive motors
        FrontLeftDrive = hardwareMap.get(DcMotorEx.class, "FrontLeftDrive");
        FrontRightDrive = hardwareMap.get(DcMotorEx.class, "FrontRightDrive");
        BackLeftDrive = hardwareMap.get(DcMotorEx.class, "BackLeftDrive");
        BackRightDrive = hardwareMap.get(DcMotorEx.class, "BackRightDrive");

        // set motor directions
        FrontLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        FrontRightDrive.setDirection(DcMotor.Direction.REVERSE);
        BackLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        BackRightDrive.setDirection(DcMotor.Direction.REVERSE);

        // set motors to RUN_TO_POSITION
        /*
        FrontLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FrontRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BackLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BackRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // set PIDF coefficients (positions)
        FrontLeftDrive.setPositionPIDFCoefficients(5);
        FrontRightDrive.setPositionPIDFCoefficients(5);
        BackLeftDrive.setPositionPIDFCoefficients(5);
        BackRightDrive.setPositionPIDFCoefficients(5);
        */

        // set PIDF coefficients (velocity)
        FrontLeftDrive.setVelocityPIDFCoefficients(2, 0.5, 0, 11);
        FrontRightDrive.setVelocityPIDFCoefficients(2, 0.5, 0, 11);
        BackLeftDrive.setVelocityPIDFCoefficients(2, 0.5, 0, 11);
        BackRightDrive.setVelocityPIDFCoefficients(2, 0.5, 0, 11);

        // set motors to RUN_USING_ENCODER
        FrontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FrontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void loop() {
        // set velocity of motors
        FrontLeftDrive.setVelocity(14.26);
        FrontRightDrive.setVelocity(14.26);
        BackLeftDrive.setVelocity(14.26);
        BackRightDrive.setVelocity(14.26);
    }
}
