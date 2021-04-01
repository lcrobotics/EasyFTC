package org.firstinspires.ftc.teamcode.AutonomousOpModes;

import android.graphics.Bitmap;
import android.graphics.Color;

import com.lcrobotics.easyftclib.vision.VuforiaSuperOp;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.RobotLog;
import com.vuforia.PIXEL_FORMAT;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

/**
 *
 */
@Disabled
@Autonomous
public class PixelAssigner extends LinearOpMode {
    public VuforiaLocalizer vuforia;
    private final static String VUFORIA_KEY = "ARgYuCf/////AAABmUYfc1+dVEQsgUBCPA2kCAFRmuTRB/XUfAJzLsRyFDRg6uMMjj6EXM8YNiY5l3oTw83H+PKgfF46gctdzrln2nnVXMebpgN9ULy1cOfdSsPk0hwSZqzcY0LWCj+rPPrZ3JyQT7gf2aw7bo8ZvWedWB7skuGIjg+9cyTJdDyXmXrQ8Bo4r4siTFNTVFxg21OH/Gd8wrVJF4RqjE+kcez3MzcnE2EPCqWTNixSge5yLg+tN87/R/dMPzqHWvmjE6F6J/7/sahPt7FQ9G6tYWnV1impzZsH7T/JT6pGr2SALwHdaNjBGbYY76ZfvAxixEdob9g6qMBhKOyLg6HTP9VzRZ06ksUhErmR2K2LSkyjxBBz";
    @Override
    public void runOpMode() {

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine

        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // This is necessary for getting pixels (integral image goal detection, etc)
        boolean[] results = vuforia.enableConvertFrameToFormat(PIXEL_FORMAT.RGB565, PIXEL_FORMAT.YUV);
        if (!results[0]) {
            // Failed to get Vuforia to convert to RGB565.
            throw new RuntimeException("Unable to convince Vuforia to generate RGB565 frames!");
        }
        vuforia.setFrameQueueCapacity(1);
        vuforia.enableConvertFrameToBitmap();
        VuforiaLocalizer.CloseableFrame frame = null;
        try {
            frame = vuforia
                    .getFrameQueue()
                    .take();
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        if (frame == null) {
            return;
        }
        // convert frame to bitmap
        Bitmap bmp = vuforia.convertFrameToBitmap(frame);

        frame.close();
        assert bmp != null;
        try {
            File file = AppUtil.getInstance().getSettingsFile("BaseImage.png");
            FileOutputStream out = new FileOutputStream(file);
            // save bitmap as png to file
            bmp.compress(Bitmap.CompressFormat.PNG, 100, out);
        } catch (IOException e) {
            e.printStackTrace();
        }
        int ceiling = 10;
        int floor = 5;
        List<int[]> redPixels = new ArrayList<>();
        for (int x = 0; x < bmp.getWidth(); x++) {
            for (int y = 0; y < bmp.getHeight(); y++) {
                int color = bmp.getPixel(x, y);
                float[] hsv = new float[3];
                Color.colorToHSV(color, hsv);
                if (hsv[0] <= ceiling && hsv[0] >= floor) {
                    redPixels.add(new int[]{x, y});
                    RobotLog.ii("HAX", String.format(Locale.US, "Pixel at (%d, %d) has a hue value of %f", x, y, hsv[0]));
                }
            }
        }
        Bitmap copy = Bitmap.createBitmap(bmp);
        transformRedPixels(bmp, redPixels);
        try {
            File file = AppUtil.getInstance().getSettingsFile("FilterImage.png");
            FileOutputStream out = new FileOutputStream(file);
            // save bitmap as png to file
            bmp.compress(Bitmap.CompressFormat.PNG, 100, out);
        } catch (IOException e) {
            e.printStackTrace();
        }
        redPixels.clear();
        int threshold = 15;
        final int RED_THRESHOLD = 120;
        for (int x = 0; x < copy.getWidth(); x++) {
            for (int y = 0; y < copy.getHeight(); y++) {
                int color = copy.getPixel(x, y);
                int r = Color.red(color);
                int g = Color.green(color);
                int b = Color.blue(color);

                if (r > Math.max(g, b) * 2 && Math.abs(g-b) < threshold && r > RED_THRESHOLD) {
                    redPixels.add(new int[]{x, y});
                }
            }
        }
        transformRedPixels(copy, redPixels);
        try {
            File file = AppUtil.getInstance().getSettingsFile("FilterImage2.png");
            FileOutputStream out = new FileOutputStream(file);
            // save bitmap as png to file
            copy.compress(Bitmap.CompressFormat.PNG, 100, out);
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    void transformRedPixels(Bitmap bm, List<int[]> redPixels) {
        for(int[] coords : redPixels) {
            bm.setPixel(coords[0], coords[1], Color.rgb(0, 177, 64)); // This will set each pixel caught to a new random color
        }
        bm.isMutable();
    }
}
