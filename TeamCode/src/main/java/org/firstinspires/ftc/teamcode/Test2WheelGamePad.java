package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class Test2WheelGamePad extends Test2Wheel {

    @Override
    public void init() {
        super.init();
    }

    @Override
    public void loop() {
        // drive
        wheels.setPower(gamepad1.right_stick_x, -gamepad1.left_stick_y);
    }
}
