package org.firstinspires.ftc.teamcode.AutonomousOpModes;

import android.graphics.Bitmap;

import com.lcrobotics.easyftclib.vision.VuforiaSuperOp;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

import java.util.concurrent.TimeUnit;

@Autonomous
public class FindPost extends VuforiaSuperOp {
    // width of rectangle to sample average red from
    final int SAMPLE_WIDTH = 10;
    // height of rectangle to sample average red from
    final int SAMPLE_HEIGHT = 10;
    // amount of pixels to skip in between samples
    final int SAMPLE_SKIP = SAMPLE_HEIGHT / 2;

    int firstX = 100;
    int firstY = 100;
    int imgHeight;
    int imgWidth;
    @Override
    public void init() {
        super.init();
        vuforia.enableConvertFrameToBitmap();
    }

    @Override
    public void init_loop() {
        try {
            VuforiaLocalizer.CloseableFrame frame = vuforia
                    .getFrameQueue()
                    .poll(10, TimeUnit.MILLISECONDS);

            Bitmap bmp = vuforia.convertFrameToBitmap(frame);
            frame.close();
            assert bmp != null;
            imgHeight = bmp.getHeight();
            imgWidth = bmp.getWidth();




        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }

    @Override
    public void loop() {

    }
}
