package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous
public class Test4WheelAuto extends Test4Wheel {


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
            stopMotors();
        }

    }

    private void stopMotors() {
        wheels.setPower(0, 0, 0);
    }

    public void spin() {
        wheels.setPower(0, 0, 0.5);
    }
}
