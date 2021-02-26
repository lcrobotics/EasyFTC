package org.firstinspires.ftc.teamcode.AutonomousOpModes;

import android.graphics.Bitmap;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

import java.util.concurrent.TimeUnit;

@Autonomous
public class FindPost extends VuforiaSuperOp {
    // where the post starts from the top of the image
    final double TOP_RECT_RATIO = 0.3;

    @Override
    public void init() {
        super.init();
    }

    @Override
    public void init_loop() {
        try {
            vuforia.enableConvertFrameToBitmap();

            VuforiaLocalizer.CloseableFrame frame = vuforia
                    .getFrameQueue()
                    .poll(10, TimeUnit.MILLISECONDS);

            Bitmap bmp = vuforia.convertFrameToBitmap(frame);
            frame.close();
            assert bmp != null;
            int imgHeight = bmp.getHeight();
            int imgWidth = bmp.getWidth();



        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }

    @Override
    public void loop() {

    }
}
