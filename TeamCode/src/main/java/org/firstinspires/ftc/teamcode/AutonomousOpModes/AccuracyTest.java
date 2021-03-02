package org.firstinspires.ftc.teamcode.AutonomousOpModes;

import android.graphics.Bitmap;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.graphics.Rect;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

import java.io.FileOutputStream;
import java.io.IOException;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

@Autonomous
public class AccuracyTest extends VuforiaSuperOp {
    ScheduledExecutorService imageSaverThread;
    int count = 0;
    @Override
    public void init() {
        super.init();
        imageSaverThread = Executors.newSingleThreadScheduledExecutor();
        vuforia.enableConvertFrameToBitmap();
        Runnable imageThings = () -> {
            if (count == 0) {
                imageSaverThread.shutdownNow();
            }
            try {
                int postWidth = 100;
                int postHeight = 200;
                frameGetter.updateFrame();
                frameGetter.updateMaxRect(0, postWidth, postHeight);
                // get vuforia frame
                VuforiaLocalizer.CloseableFrame frame = vuforia
                        .getFrameQueue()
                        .poll(10, TimeUnit.MILLISECONDS);

                Bitmap bmp = vuforia.convertFrameToBitmap(frame);
                frame.close();
                // paint object for drawing style
                Paint paint = new Paint();
                paint.setColor(Color.GREEN);
                paint.setStyle(Paint.Style.STROKE);
                paint.setStrokeWidth(10);
                // canvas to draw on bitmap
                assert bmp != null;
                Canvas canvas = new Canvas(bmp);

                int bottom = frameGetter.yMax + postHeight;
                int right = frameGetter.xMax + postWidth;
                canvas.drawRect(new Rect(frameGetter.xMax, frameGetter.yMax, right, bottom), paint);

                FileOutputStream out = new FileOutputStream("ACCURACY_TEST" + count);
                // save bitmap as png to file
                bmp.compress(Bitmap.CompressFormat.PNG, 100, out);

                out.close();
            } catch (InterruptedException | IOException e) {
                e.printStackTrace();
            }
            count++;
        };

        imageSaverThread.scheduleAtFixedRate(imageThings, 0, 1, TimeUnit.SECONDS);
    }

    @Override
    public void loop() {
        telemetry.addData("Image count", count);
    }

    // @Override
    // public void stop() {
        // imageSaverThread.shutdownNow();
    // }
}
