package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;

public class Test2WheelAuto extends Test2Wheel {

    public boolean first = true;

    public ElapsedTime timer;

    @Override
    public void init() {
        super.init();
        timer = new ElapsedTime();
    }

    @Override
    public void loop() {
        if (first) {
            timer.reset();
            first = !first;
        }
        if (timer.seconds() < 5) {
            spin();
        } else {
            halt();
        }
    }

    private void halt() {
        wheels.setPower(0, 0);
    }

    private void spin() {
        wheels.setPower(1, 1);
    }
}
