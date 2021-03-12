package org.firstinspires.ftc.teamcode.AutonomousOpModes;

import android.graphics.Bitmap;

import com.lcrobotics.easyftclib.vision.VuforiaSuperOp;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

import java.io.FileOutputStream;
import java.io.IOException;
import java.util.concurrent.TimeUnit;

@Autonomous
public class GetImage extends VuforiaSuperOp {

    @Override
    public void init() {
        super.init();

        vuforia.enableConvertFrameToBitmap();
        VuforiaLocalizer.CloseableFrame frame = null;
        try {
            frame = vuforia
                    .getFrameQueue()
                    .poll(10, TimeUnit.MILLISECONDS);

            Bitmap bmp = vuforia.convertFrameToBitmap(frame);
            frame.close();

            FileOutputStream out = new FileOutputStream("IMAGE");
            // save bitmap as png to file
            bmp.compress(Bitmap.CompressFormat.PNG, 100, out);

            out.close();
        } catch (InterruptedException | IOException e) {
            e.printStackTrace();
        }
    }

    @Override
    public void loop() {

    }
}
