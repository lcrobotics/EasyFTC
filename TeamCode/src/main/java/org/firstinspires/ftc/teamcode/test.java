package org.firstinspires.ftc.teamcode;


import com.lcrobotics.easyftclib.AdvancedOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="TEST", group="Iterative Opmode")
public class test extends AdvancedOpMode {


    DcMotor frontRightDrive;
    DcMotor frontLeftDrive;
    DcMotor backLeftDrive;
    DcMotor backRightDrive;
    
    @Override
    public void init() {
        frontLeftDrive = hardwareMap.get(DcMotor.class, "FrontLeftDrive");
        frontRightDrive = hardwareMap.get(DcMotor.class, "FrontRightDrive");
        backLeftDrive = hardwareMap.get(DcMotor.class, "BackLeftDrive");
        backRightDrive = hardwareMap.get(DcMotor.class, "BackRightDrive");

}

    @Override
    public void loop() {

        float x = gamepad1.left_stick_x;
        float y = gamepad1.left_stick_y;
        frontRightDrive.setPower(Range.clip(x - y, -1, 1));
        backRightDrive.setPower(Range.clip(x - y, -1, 1));
        backLeftDrive.setPower(Range.clip(x + y, -1, 1));
        frontLeftDrive.setPower(Range.clip(x + y, -1, 1));
    }
}
