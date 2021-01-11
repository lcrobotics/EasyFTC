package org.firstinspires.ftc.teamcode;

import com.lcrobotics.easyftclib.commandCenter.hardware.Motor;
import com.lcrobotics.easyftclib.commandCenter.hardware.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

@TeleOp
public class MotorTest extends OpMode {

    MotorEx frontLeftDrive;
    MotorEx frontRightDrive;
    MotorEx backLeftDrive;
    MotorEx backRightDrive;

    final int CPR = 448;
    final int MAX_RPM = 64;

    @Override
    public void init() {

        frontRightDrive = new MotorEx(hardwareMap, "FrontRightDrive", CPR, MAX_RPM);
        frontLeftDrive = new MotorEx(hardwareMap, "FrontLeftDrive", CPR, MAX_RPM);
        backLeftDrive = new MotorEx(hardwareMap, "BackLeftDrive", CPR, MAX_RPM);
        backRightDrive = new MotorEx(hardwareMap, "BackRightDrive", CPR, MAX_RPM);

        frontRightDrive.setInverted(true);
        backRightDrive.setInverted(true);


        frontRightDrive.setRunMode(Motor.RunMode.VelocityControl);
        frontLeftDrive.setRunMode(Motor.RunMode.VelocityControl);
        frontRightDrive.setRunMode(Motor.RunMode.VelocityControl);
        backLeftDrive.setRunMode(Motor.RunMode.VelocityControl);

        frontRightDrive.setPositionCoefficient(5);
        frontLeftDrive.setPositionCoefficient(5);
        backLeftDrive.setPositionCoefficient(5);
        backRightDrive.setPositionCoefficient(5);

        frontRightDrive.setVelocityCoefficients(2.0, 0.5, 1);
        frontLeftDrive.setVelocityCoefficients(2.0, 0.5, 1);
        backLeftDrive.setVelocityCoefficients(2.0, 0.5, 1);
        backRightDrive.setVelocityCoefficients(2.0, 0.5, 1);

    }

    @Override
    public void loop() {
        drive(-gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x);
    }
    public void drive(double x, double y, double w) {
        frontLeftDrive.set(Range.clip(y - x + w, -1, 1));
        frontRightDrive.set(Range.clip(y + x - w, -1, 1));
        backLeftDrive.set(Range.clip(y + x + w, -1, 1));
        backRightDrive.set(Range.clip(y - x - w, -1, 1));

    }
}
