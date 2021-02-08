package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous
public class telemetryTest extends OpMode {
    public void init() {
        telemetry.addData("reeeeeeeeeeeeee", "help");

        try {
            Thread.sleep(2000);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }

    @Override
    public void loop() {
        telemetry.addData("help", "reeeeeeeeeeeeeeeeee");
    }
}
