package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Test4WheelGamepad", group="Test")
public class Test4WheelGamePad extends Test4Wheel {
    @Override
    public void init() {
        super.init();
    }

    @Override
    public void loop() {
        wheels.setPower(-gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x);
    }
}
