package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp
public class CharlieTest extends OpMode {


    DcMotor frontRightDrive;
    DcMotor frontLeftDrive;
    DcMotor backLeftDrive;
    DcMotor backRightDrive;

    Servo GrabServo;
    DcMotor GrabMotor;

    boolean servo = false;

    @Override
    public void init() {
        // drive motors
        frontLeftDrive = hardwareMap.get(DcMotor.class, "FrontLeftDrive");
        frontRightDrive = hardwareMap.get(DcMotor.class, "FrontRightDrive");
        backLeftDrive = hardwareMap.get(DcMotor.class, "BackLeftDrive");
        backRightDrive = hardwareMap.get(DcMotor.class, "BackRightDrive");
        // grabbing motors and servos
        GrabMotor = hardwareMap.get(DcMotor.class, "GrabMotor");
        GrabServo = hardwareMap.get(Servo.class, "GrabServo");
        // reverse right drive motors
        frontRightDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        GrabMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        GrabMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void loop() {
        telemetry.addData("GrabMotor: ", GrabMotor.getPower());
        telemetry.addData("GrabServo: ", GrabServo.getPosition());
//        telemetry.addData("A button: ", gamepad1.a);
//        telemetry.addData("B button: ", gamepad1.b);
//        telemetry.addData("X button: ", gamepad1.x);
//        telemetry.addData("Y button: ", gamepad1.y);
        // bind a and b buttons to grab motor
        double positive  = 0.5 * gamepad1.right_trigger;
        double negative = 0.5 * gamepad1.left_trigger;
        // bind x and y buttons to grab servo
        if (gamepad1.x) {
            servo = true;
        }
        if (gamepad1.y) {
            servo = false;
        }

        GrabMotor.setPower(gamepad1.right_trigger - gamepad1.left_trigger);
        GrabServo.setPosition(servo? 0 : 1);
        // drive using controller sticks
        drive(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);
    }

    private void drive(double x, double y, double w) {
        // calculate power for each wheel
        double frontLeftPow = Range.clip(0.5 * (y - x + w), -1, 1);
        double frontRightPow = Range.clip(0.5 * (y + x - w), -1, 1);
        double backLeftPow = Range.clip(0.5 * (y + x + w), -1, 1);
        double backRightPow = Range.clip(0.5 * (y - x - w), -1, 1);

        frontLeftDrive.setPower(frontLeftPow);
        frontRightDrive.setPower(frontRightPow);
        backLeftDrive.setPower(backLeftPow);
        backRightDrive.setPower(backRightPow);
    }
}
