package org.firstinspires.ftc.teamcode.AutonomousOpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;

import android.graphics.Bitmap;
import android.graphics.Color;
import android.graphics.Paint;
import android.graphics.RectF;
import android.graphics.Canvas;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

import java.nio.ByteBuffer;
import java.util.concurrent.BlockingQueue;

@Autonomous
public class RectTest extends OpMode {

    static final Paint paint = new Paint();
    VuforiaLocalizer vuforia;
    private final static String VUFORIA_KEY = "ARgYuCf/////AAABmUYfc1+dVEQsgUBCPA2kCAFRmuTRB/XUfAJzLsRyFDRg6uMMjj6EXM8YNiY5l3oTw83H+PKgfF46gctdzrln2nnVXMebpgN9ULy1cOfdSsPk0hwSZqzcY0LWCj+rPPrZ3JyQT7gf2aw7bo8ZvWedWB7skuGIjg+9cyTJdDyXmXrQ8Bo4r4siTFNTVFxg21OH/Gd8wrVJF4RqjE+kcez3MzcnE2EPCqWTNixSge5yLg+tN87/R/dMPzqHWvmjE6F6J/7/sahPt7FQ9G6tYWnV1impzZsH7T/JT6pGr2SALwHdaNjBGbYY76ZfvAxixEdob9g6qMBhKOyLg6HTP9VzRZ06ksUhErmR2K2LSkyjxBBz";
    private BlockingQueue<VuforiaLocalizer.CloseableFrame> frameQueue;
    float[] size;

    public void init(){

        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");
        //parameters.useExtendedTracking = false;
        parameters.useExtendedTracking = true;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        boolean[] results = vuforia.enableConvertFrameToFormat(PIXEL_FORMAT.RGB565, PIXEL_FORMAT.YUV);
        if (!results[0]) { // Failed to get Vuforia to convert to RGB565.
            throw new RuntimeException("Unable to convince Vuforia to generate RGB565 frames!");
        }

        vuforia.setFrameQueueCapacity(1);
        frameQueue = vuforia.getFrameQueue();

        paint.setColor(Color.RED);
        paint.setStyle(Paint.Style.STROKE);
        paint.setStrokeWidth(10);
        size = vuforia.getCameraCalibration().getSize().getData();

    }

    public void loop(){
        float left = 100, top = 100, right = 200, bottom = 200;
        RectF location = new RectF(left, top, right, bottom);
        VuforiaLocalizer.CloseableFrame vuforiaFrame = null;
        try {
            vuforiaFrame = frameQueue.take();
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        Bitmap copiedBitmap;
        for (int i = 0; i < vuforiaFrame.getNumImages(); i++) {
            Image image = vuforiaFrame.getImage(i);
            if (image.getFormat() == PIXEL_FORMAT.RGB565) {
                ByteBuffer rgb565ByteBuffer = image.getPixels();
                Bitmap rgb565Bitmap = Bitmap.createBitmap((int)size[0], (int)size[1], Bitmap.Config.RGB_565);
                rgb565Bitmap.copyPixelsFromBuffer(rgb565ByteBuffer.duplicate());
                copiedBitmap = rgb565Bitmap.copy(rgb565Bitmap.getConfig(), true);
                Canvas canvas = new Canvas(copiedBitmap);
                canvas.drawRect(location, paint);
            }
        }

    }
}
