package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class ZackTest extends SuperOp {

    @Override
    public void init() {
        super.init();
        telemetry.addData("Drive Train: ", wheels);

    }

    @Override
    public void loop() {
        driveWithVectors(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        telemetry.update();
    }

    public void driveWithVectors(double vertical, double horizontal, double rotational)
    {
        double magnitude = Math.sqrt((horizontal*horizontal) + (vertical*vertical));
        double angle = Math.atan2(horizontal, -vertical) + (Math.PI / 4);

        double FL_BR = Math.sin(angle) * magnitude;
        double FR_BL = Math.cos(angle) * magnitude;

        FrontLeftDrive.setPower(FL_BR + rotational);
        FrontRightDrive.setPower(FR_BL - rotational);
        BackLeftDrive.setPower(FR_BL + rotational);
        BackRightDrive.setPower(FL_BR - rotational);
    }
}
