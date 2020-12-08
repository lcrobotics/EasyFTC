package org.firstinspires.ftc.teamcode;

import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "TeleopTest")
public class SoundTest extends LinearOpMode {


    private boolean wasA = false;
    private boolean isA = false;
    private boolean rickFound;

    @Override
    public void runOpMode() throws InterruptedException {
        int rickID = hardwareMap.appContext.getResources().getIdentifier("rick", "raw", hardwareMap.appContext.getPackageName());

        if (rickID != 0) {
            rickFound = SoundPlayer.getInstance().preload(hardwareMap.appContext, rickID);
        }

        waitForStart();

        while (opModeIsActive()) {

            // say Silver each time gamepad X is pressed (This sound is a resource)
            if (rickFound && (isA = gamepad1.a) && !wasA) {
                SoundPlayer.getInstance().startPlaying(hardwareMap.appContext, rickID);
            }
            // Save last button states
            wasA = isA;
        }
    }
}
